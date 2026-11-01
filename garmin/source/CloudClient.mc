import Toybox.Application.Storage;
import Toybox.Communications;
import Toybox.Lang;
import Toybox.System;
import Toybox.Time;
import Toybox.Timer;

// One HTTPS call at a time, through the phone. API 5.2 sends a JSON PUT by
// putting the dictionary in the parameters and setting REQUEST_CONTENT_TYPE_JSON.
class CloudClient {
    static const ERR_NONE = 0;
    static const ERR_PHONE = 1;
    static const ERR_NETWORK = 2;
    static const ERR_AUTH = 3;
    static const ERR_NOT_FOUND = 4;
    static const ERR_PROPERTIES = 5;
    static const ERR_HTTP = 6;
    static const ERR_PARSE = 7;
    static const ERR_UNCONFIRMED = 8;
    static const ERR_UNCHANGED = 9;

    static const TOKEN_URL = "https://api2.arduino.cc/iot/v1/clients/token";
    static const API_ROOT = "https://api2.arduino.cc/iot/v2/things/";

    var _busy as Boolean;
    var _callback as Method?;
    var _goalPublish as Boolean;
    var _didPublish as Boolean;
    var _publishAccepted as Boolean;
    var _authRetried as Boolean;
    var _desiredOpen as Boolean;
    var _error as Number;
    var _token as String?;
    var _tokenExpiry as Number?;
    var _nivelId as String?;
    var _valvulaId as String?;
    var _nivel as Number?;
    var _valvula as Boolean?;
    var _timer as Timer.Timer?;
    var _deferred as Number;

    function initialize() {
        _busy = false;
        _callback = null;
        _goalPublish = false;
        _didPublish = false;
        _publishAccepted = false;
        _authRetried = false;
        _desiredOpen = false;
        _error = ERR_NONE;
        _token = storedString("token");
        _tokenExpiry = numericValue(Storage.getValue("token_expiry"));
        _nivelId = storedString("nivel_id");
        _valvulaId = storedString("valvula_id");
        _nivel = numericValue(Storage.getValue("nivel"));
        var valve = Storage.getValue("valvula");
        _valvula = (valve instanceof Boolean) ? valve : null;
        _timer = null;
        _deferred = 0;
    }

    function isBusy() as Boolean {
        return _busy;
    }

    function isPublishing() as Boolean {
        return _busy && _goalPublish;
    }

    function nivel() as Number? {
        return _nivel;
    }

    function valvula() as Boolean? {
        return _valvula;
    }

    function refresh(callback as Method) as Void {
        if (_busy) {
            return;
        }
        start(callback, false);
    }

    function publishValve(abrir as Boolean, callback as Method) as Void {
        if (_busy) {
            return;
        }
        _desiredOpen = abrir;
        start(callback, true);
    }

    function start(callback as Method, publish as Boolean) as Void {
        _busy = true;
        _callback = callback;
        _goalPublish = publish;
        _didPublish = false;
        _publishAccepted = false;
        _authRetried = false;
        _error = ERR_NONE;
        kick();
    }

    function kick() as Void {
        if (!tokenValid()) {
            requestToken();
        } else if (_goalPublish && !_didPublish) {
            if (hasIds()) {
                requestPublish();
            } else {
                requestList();
            }
        } else {
            requestList();
        }
    }

    // The next request has to start after this callback returns. A second
    // makeWebRequest inside the callback is still "in progress" on the watch.
    function queue(step as Number) as Void {
        _deferred = step;
        _timer = new Timer.Timer();
        _timer.start(method(:onQueued), 100, false);
    }

    function onQueued() as Void {
        if (_callback == null) {
            return;
        }
        if (_deferred == 1) {
            requestPublish();
        } else if (_deferred == 2) {
            requestList();
        } else if (_deferred == 3) {
            requestToken();
        } else {
            kick();
        }
    }

    function send(url as String, params as Dictionary or Null, options as Dictionary, callback as Method) as Void {
        try {
            Communications.makeWebRequest(url, params, options, callback);
        } catch (ex) {
            ex.printStackTrace();
            _error = ERR_NETWORK;
            finish(false);
        }
    }

    function tokenValid() as Boolean {
        if (!(_token instanceof String) || _token.length() == 0) {
            return false;
        }
        if (!(_tokenExpiry instanceof Number)) {
            return false;
        }
        var now = numericValue(Time.now().value());
        if (now == null) {
            return false;
        }
        return now < _tokenExpiry;
    }

    function hasIds() as Boolean {
        if (!(_nivelId instanceof String) || !(_valvulaId instanceof String)) {
            return false;
        }
        return (_nivelId.length() > 0) && (_valvulaId.length() > 0);
    }

    function requestToken() as Void {
        send(
            TOKEN_URL,
            {
                "grant_type" => "client_credentials",
                "client_id" => Secrets.CLIENT_ID,
                "client_secret" => Secrets.CLIENT_SECRET,
                "audience" => "https://api2.arduino.cc/iot"
            },
            {
                :method => Communications.HTTP_REQUEST_METHOD_POST,
                :headers => {
                    "Accept" => "application/json"
                },
                :responseType => Communications.HTTP_RESPONSE_CONTENT_TYPE_JSON
            },
            method(:onToken)
        );
    }

    function onToken(responseCode as Number, data as Object or Null) as Void {
        if (_callback == null) {
            return;
        }
        System.println("cloud token " + responseCode);
        if (!httpOk(responseCode) || !(data instanceof Dictionary)) {
            _error = (responseCode == 401 || responseCode == 403) ? ERR_AUTH : classify(responseCode);
            finish(false);
            return;
        }
        var access = data["access_token"];
        if (!(access instanceof String) || access.length() == 0) {
            _error = ERR_PARSE;
            finish(false);
            return;
        }
        _token = access;
        var expires = numericValue(data["expires_in"]);
        if (expires == null || expires < 120) {
            expires = 300;
        }
        var now = numericValue(Time.now().value());
        _tokenExpiry = (now == null) ? expires : now + expires - 60;
        Storage.setValue("token", _token);
        Storage.setValue("token_expiry", _tokenExpiry);
        queue(0);
    }

    function requestList() as Void {
        send(
            propertiesUrl(),
            null,
            {
                :method => Communications.HTTP_REQUEST_METHOD_GET,
                :headers => authHeaders(),
                :responseType => Communications.HTTP_RESPONSE_CONTENT_TYPE_JSON
            },
            method(:onList)
        );
    }

    function onList(responseCode as Number, data as Object or Null) as Void {
        if (_callback == null) {
            return;
        }
        System.println("cloud list " + responseCode);
        if (responseCode == 401 && !_authRetried) {
            retryAuth();
            return;
        }
        if (!httpOk(responseCode) || !(data instanceof Array)) {
            if (_publishAccepted) {
                _error = ERR_UNCONFIRMED;
            } else if (httpOk(responseCode)) {
                _error = ERR_PARSE;
            } else {
                _error = classify(responseCode);
            }
            finish(false);
            return;
        }
        if (!parseProperties(data)) {
            _error = ERR_PROPERTIES;
            finish(false);
            return;
        }
        if (_goalPublish && !_didPublish) {
            if (!hasIds()) {
                _error = ERR_PROPERTIES;
                finish(false);
                return;
            }
            queue(1);
            return;
        }
        if (_goalPublish && _didPublish && _publishAccepted) {
            if (!(_valvula instanceof Boolean) || (_valvula != _desiredOpen)) {
                _valvula = _desiredOpen;
                Storage.setValue("valvula", _valvula);
            }
            finish(true);
            return;
        }
        if (_goalPublish && _didPublish && (_valvula instanceof Boolean) && (_valvula != _desiredOpen)) {
            _error = ERR_UNCHANGED;
            finish(false);
            return;
        }
        finish(true);
    }

    function requestPublish() as Void {
        var url = propertiesUrl() + "/" + _valvulaId + "/publish";
        var headers = authHeaders();
        headers["Content-Type"] = Communications.REQUEST_CONTENT_TYPE_JSON;
        // Arduino applies JSON true/false and the numbers 1/0. It answers
        // 200 and ignores the strings "true" and "false". The watch encodes
        // a boolean as that string, so the valve never moves.
        send(
            url,
            {
                "value" => _desiredOpen ? 1 : 0
            },
            {
                :method => Communications.HTTP_REQUEST_METHOD_PUT,
                :headers => headers,
                :responseType => Communications.HTTP_RESPONSE_CONTENT_TYPE_TEXT_PLAIN
            },
            method(:onPublish)
        );
    }

    function onPublish(responseCode as Number, data as Object or Null) as Void {
        if (_callback == null) {
            return;
        }
        System.println("cloud publish " + responseCode + ((data == null) ? " empty" : " body"));
        if (responseCode == 401 && !_authRetried) {
            retryAuth();
            return;
        }
        _didPublish = true;
        if (httpOk(responseCode)) {
            _publishAccepted = true;
            _valvula = _desiredOpen;
            Storage.setValue("valvula", _valvula);
            queue(2);
            return;
        }
        // An empty or non-JSON body still means the write may have landed.
        if (responseCode == Communications.INVALID_HTTP_BODY_IN_NETWORK_RESPONSE) {
            queue(2);
            return;
        }
        _error = classify(responseCode);
        finish(false);
    }

    function retryAuth() as Void {
        _authRetried = true;
        _token = null;
        _tokenExpiry = null;
        Storage.deleteValue("token");
        Storage.deleteValue("token_expiry");
        queue(3);
    }

    function parseProperties(items as Array) as Boolean {
        var foundNivel = false;
        var foundValve = false;
        for (var i = 0; i < items.size(); i++) {
            var prop = items[i];
            if (prop instanceof Dictionary) {
                if (matches(prop, "nivel_percentual")) {
                    var nivelId = prop["id"];
                    if (nivelId instanceof String) {
                        _nivelId = nivelId;
                        Storage.setValue("nivel_id", _nivelId);
                    }
                    var nivel = numericValue(prop["last_value"]);
                    if (nivel != null) {
                        _nivel = nivel;
                        Storage.setValue("nivel", _nivel);
                    }
                    foundNivel = true;
                } else if (matches(prop, "abrir_valvula")) {
                    var valveId = prop["id"];
                    if (valveId instanceof String) {
                        _valvulaId = valveId;
                        Storage.setValue("valvula_id", _valvulaId);
                    }
                    var valve = asBoolean(prop["last_value"]);
                    if (valve != null) {
                        _valvula = valve;
                        Storage.setValue("valvula", _valvula);
                    }
                    foundValve = true;
                }
            }
        }
        return foundNivel && foundValve;
    }

    function matches(prop as Dictionary, expected as String) as Boolean {
        var variableName = prop["variable_name"];
        if (variableName instanceof String && variableName.equals(expected)) {
            return true;
        }
        var name = prop["name"];
        return (name instanceof String) && name.equals(expected);
    }

    function propertiesUrl() as String {
        return API_ROOT + Secrets.THING_ID + "/properties";
    }

    function authHeaders() as Dictionary {
        return {
            "Authorization" => "Bearer " + _token,
            "Accept" => "application/json"
        };
    }

    function finish(ok as Boolean) as Void {
        if (ok) {
            _error = ERR_NONE;
        }
        var cached = !ok && (_nivel != null);
        var callback = _callback;
        _callback = null;
        _busy = false;
        _goalPublish = false;
        if (callback != null) {
            callback.invoke({
                :ok => ok,
                :nivel => _nivel,
                :valvula => _valvula,
                :error => _error,
                :cached => cached
            });
        }
    }

    function classify(code as Number) as Number {
        if (code == Communications.BLE_CONNECTION_UNAVAILABLE) {
            return ERR_PHONE;
        }
        if (code == Communications.INVALID_HTTP_BODY_IN_NETWORK_RESPONSE) {
            return ERR_PARSE;
        }
        if (code == 401 || code == 403) {
            return ERR_AUTH;
        }
        if (code == 404) {
            return ERR_NOT_FOUND;
        }
        if (code < 0) {
            return ERR_NETWORK;
        }
        return ERR_HTTP;
    }

    function httpOk(code as Number) as Boolean {
        return code >= 200 && code < 300;
    }

    function storedString(key as String) as String? {
        var value = Storage.getValue(key);
        if (value instanceof String && value.length() > 0) {
            return value;
        }
        return null;
    }

    function numericValue(value as Object or Null) as Number? {
        if (value == null) {
            return null;
        }
        if (value instanceof Number) {
            return value;
        }
        if (value instanceof Float) {
            return value.toNumber();
        }
        if (value instanceof Long) {
            return value.toNumber();
        }
        if (value instanceof String) {
            if (value.equals("N/A") || value.length() == 0) {
                return null;
            }
            var parsed = value.toNumber();
            if (parsed != null) {
                return parsed;
            }
            var asFloat = value.toFloat();
            if (asFloat != null) {
                return asFloat.toNumber();
            }
        }
        return null;
    }

    function asBoolean(value as Object or Null) as Boolean? {
        if (value instanceof Boolean) {
            return value;
        }
        if (value instanceof Number) {
            return value != 0;
        }
        if (value instanceof Float) {
            return value != 0.0;
        }
        if (value instanceof String) {
            if (value.equals("true") || value.equals("1")) {
                return true;
            }
            if (value.equals("false") || value.equals("0")) {
                return false;
            }
        }
        return null;
    }
}
