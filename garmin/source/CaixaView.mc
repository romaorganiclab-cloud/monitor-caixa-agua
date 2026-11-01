import Toybox.Application.Storage;
import Toybox.Graphics;
import Toybox.Lang;
import Toybox.System;
import Toybox.WatchUi;

class CaixaView extends WatchUi.View {
    var _client as CloudClient;
    var _nivel as Number?;
    var _valvula as Boolean?;
    var _message as String;
    var _confirming as Boolean;
    var _cached as Boolean;
    var _width as Number;
    var _height as Number;
    var _yesX as Number;
    var _noX as Number;
    var _btnY as Number;
    var _btnW as Number;
    var _btnH as Number;
    var _valveX as Number;
    var _valveY as Number;
    var _valveW as Number;
    var _valveH as Number;
    var _pendingAbrir as Boolean?;

    var _abrirNaciente as String;
    var _fecharNaciente as String;
    var _naciente as String;
    var _aberta as String;
    var _fechada as String;
    var _abrirPergunta as String;
    var _fecharPergunta as String;
    var _aviso1 as String;
    var _aviso2 as String;
    var _sim as String;
    var _nao as String;
    var _semTelefone as String;
    var _semRede as String;
    var _acessoNegado as String;
    var _naoEncontrado as String;
    var _variavelAusente as String;
    var _respostaInvalida as String;
    var _falhaHttp as String;
    var _atualizando as String;
    var _enviando as String;
    var _semConfirmacao as String;
    var _semLeitura as String;
    var _toqueAlternar as String;
    var _naoMudou as String;

    function initialize() {
        View.initialize();
        _client = new CloudClient();
        _nivel = _client.nivel();
        _valvula = _client.valvula();
        _message = "";
        _confirming = false;
        _cached = false;
        _pendingAbrir = null;
        var settings = System.getDeviceSettings();
        layout(settings.screenWidth, settings.screenHeight);
        _abrirNaciente = WatchUi.loadResource(Rez.Strings.AbrirNaciente);
        _fecharNaciente = WatchUi.loadResource(Rez.Strings.FecharNaciente);
        _naciente = WatchUi.loadResource(Rez.Strings.Naciente);
        _aberta = WatchUi.loadResource(Rez.Strings.Aberta);
        _fechada = WatchUi.loadResource(Rez.Strings.Fechada);
        _abrirPergunta = WatchUi.loadResource(Rez.Strings.AbrirPergunta);
        _fecharPergunta = WatchUi.loadResource(Rez.Strings.FecharPergunta);
        _aviso1 = WatchUi.loadResource(Rez.Strings.AvisoLinha1);
        _aviso2 = WatchUi.loadResource(Rez.Strings.AvisoLinha2);
        _sim = WatchUi.loadResource(Rez.Strings.Sim);
        _nao = WatchUi.loadResource(Rez.Strings.Nao);
        _semTelefone = WatchUi.loadResource(Rez.Strings.SemTelefone);
        _semRede = WatchUi.loadResource(Rez.Strings.SemRede);
        _acessoNegado = WatchUi.loadResource(Rez.Strings.AcessoNegado);
        _naoEncontrado = WatchUi.loadResource(Rez.Strings.NaoEncontrado);
        _variavelAusente = WatchUi.loadResource(Rez.Strings.VariavelAusente);
        _respostaInvalida = WatchUi.loadResource(Rez.Strings.RespostaInvalida);
        _falhaHttp = WatchUi.loadResource(Rez.Strings.FalhaHttp);
        _atualizando = WatchUi.loadResource(Rez.Strings.Atualizando);
        _enviando = WatchUi.loadResource(Rez.Strings.Enviando);
        _semConfirmacao = WatchUi.loadResource(Rez.Strings.SemConfirmacao);
        _semLeitura = WatchUi.loadResource(Rez.Strings.SemLeitura);
        _toqueAlternar = WatchUi.loadResource(Rez.Strings.ToqueAlternar);
        _naoMudou = WatchUi.loadResource(Rez.Strings.NaoMudou);
    }

    function layout(width as Number, height as Number) as Void {
        _width = width;
        _height = height;
        _btnW = width * 34 / 100;
        _btnH = height * 14 / 100;
        _btnY = height * 62 / 100;
        var gap = width * 4 / 100;
        _yesX = width / 2 - _btnW - gap / 2;
        _noX = width / 2 + gap / 2;
        _valveW = width * 72 / 100;
        _valveH = height * 16 / 100;
        _valveX = (width - _valveW) / 2;
        _valveY = height * 60 / 100;
    }

    function onShow() as Void {
        refresh();
    }

    function onUpdate(dc as Graphics.Dc) as Void {
        layout(dc.getWidth(), dc.getHeight());
        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_BLACK);
        dc.clear();
        if (_confirming) {
            drawConfirm(dc);
        } else {
            drawMain(dc);
        }
    }

    function drawMain(dc as Graphics.Dc) as Void {
        var number = (_nivel == null) ? "--" : _nivel.toString();
        var color = levelColor(_nivel);
        var numFont = pickNumberFont(dc, number);
        var unitFont = Graphics.FONT_MEDIUM;
        var numW = dc.getTextWidthInPixels(number, numFont);
        var unitW = dc.getTextWidthInPixels("%", unitFont);
        var startX = (_width - numW - unitW - 6) / 2;
        var numberY = _height * 34 / 100;
        dc.setColor(color, Graphics.COLOR_TRANSPARENT);
        dc.drawText(startX, numberY, numFont, number, Graphics.TEXT_JUSTIFY_LEFT | Graphics.TEXT_JUSTIFY_VCENTER);
        dc.drawText(startX + numW + 6, numberY + dc.getFontHeight(numFont) / 8, unitFont, "%", Graphics.TEXT_JUSTIFY_LEFT | Graphics.TEXT_JUSTIFY_VCENTER);

        var valveLabel = _abrirNaciente;
        var valveFill = Graphics.COLOR_DK_BLUE;
        if (_valvula == null) {
            valveLabel = _naciente;
            valveFill = Graphics.COLOR_DK_GRAY;
        } else if (_valvula) {
            valveLabel = _fecharNaciente;
            valveFill = Graphics.COLOR_DK_GRAY;
        }
        drawButton(dc, _valveX, _valveY, _valveW, _valveH, valveLabel, valveFill);

        var status = _message;
        if (status.length() == 0 && _cached) {
            status = _semTelefone;
        } else if (status.length() == 0 && (_valvula instanceof Boolean)) {
            status = _valvula ? _aberta : _fechada;
        }
        var hint = (_message.length() == 0) && !_cached;
        var statusColor = hint ? Graphics.COLOR_DK_GRAY : Graphics.COLOR_YELLOW;
        drawCentered(dc, status, _height * 82 / 100, Graphics.FONT_XTINY, statusColor);
    }

    function drawConfirm(dc as Graphics.Dc) as Void {
        var opening = !(_valvula == true);
        var question = opening ? _abrirPergunta : _fecharPergunta;
        if (_nivel != null) {
            drawCentered(dc, _nivel.toString() + "%", _height * 18 / 100, Graphics.FONT_SMALL, levelColor(_nivel));
        }
        drawCentered(dc, question, _height * 32 / 100, Graphics.FONT_SMALL, Graphics.COLOR_WHITE);
        if (opening) {
            drawCentered(dc, _aviso1, _height * 44 / 100, Graphics.FONT_TINY, Graphics.COLOR_LT_GRAY);
            drawCentered(dc, _aviso2, _height * 52 / 100, Graphics.FONT_TINY, Graphics.COLOR_LT_GRAY);
        }
        drawButton(dc, _yesX, _btnY, _btnW, _btnH, _sim, Graphics.COLOR_DK_BLUE);
        drawButton(dc, _noX, _btnY, _btnW, _btnH, _nao, Graphics.COLOR_DK_GRAY);
    }

    function drawButton(dc as Graphics.Dc, x as Number, y as Number, width as Number, height as Number, label as String, fill as Number) as Void {
        dc.setColor(fill, Graphics.COLOR_TRANSPARENT);
        dc.fillRoundedRectangle(x, y, width, height, height / 4);
        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);
        dc.drawText(x + width / 2, y + height / 2, Graphics.FONT_SMALL, label, Graphics.TEXT_JUSTIFY_CENTER | Graphics.TEXT_JUSTIFY_VCENTER);
    }

    function drawCentered(dc as Graphics.Dc, text as String, y as Number, font as Number, color as Number) as Void {
        var maxWidth = _width * 78 / 100;
        var chosen = font;
        if (dc.getTextWidthInPixels(text, chosen) > maxWidth) {
            chosen = Graphics.FONT_XTINY;
        }
        dc.setColor(color, Graphics.COLOR_TRANSPARENT);
        dc.drawText(_width / 2, y, chosen, text, Graphics.TEXT_JUSTIFY_CENTER | Graphics.TEXT_JUSTIFY_VCENTER);
    }

    function pickNumberFont(dc as Graphics.Dc, number as String) as Number {
        var maxWidth = _width * 68 / 100;
        if (dc.getTextWidthInPixels(number, Graphics.FONT_SYSTEM_NUMBER_HOT) <= maxWidth) {
            return Graphics.FONT_SYSTEM_NUMBER_HOT;
        }
        if (dc.getTextWidthInPixels(number, Graphics.FONT_NUMBER_HOT) <= maxWidth) {
            return Graphics.FONT_NUMBER_HOT;
        }
        return Graphics.FONT_NUMBER_MEDIUM;
    }

    function levelColor(nivel as Number?) as Number {
        if (nivel == null) {
            return Graphics.COLOR_WHITE;
        }
        if (nivel >= 70) {
            return Graphics.COLOR_GREEN;
        }
        if (nivel >= 30) {
            return Graphics.COLOR_YELLOW;
        }
        return Graphics.COLOR_RED;
    }

    function handleTap(x as Number, y as Number) as Boolean {
        if (_client.isPublishing()) {
            return false;
        }
        var pad = _height * 4 / 100;
        if (inside(x, y, _valveX - pad, _valveY - pad, _valveW + pad * 2, _valveH + pad * 2)) {
            confirmYes();
            return true;
        }
        return false;
    }

    function inside(x as Number, y as Number, left as Number, top as Number, width as Number, height as Number) as Boolean {
        return x >= left && x < left + width && y >= top && y < top + height;
    }

    function openConfirm() as Void {
        if (!(_valvula instanceof Boolean)) {
            _message = _semLeitura;
            WatchUi.requestUpdate();
            return;
        }
        _confirming = true;
        WatchUi.requestUpdate();
    }

    function cancelConfirm() as Void {
        _confirming = false;
        WatchUi.requestUpdate();
    }

    function confirmYes() as Void {
        if (!(_valvula instanceof Boolean) || _client.isPublishing()) {
            return;
        }
        var abrir = !_valvula;
        _confirming = false;
        _message = _enviando;
        WatchUi.requestUpdate();
        if (_client.isBusy()) {
            _pendingAbrir = abrir;
            return;
        }
        _client.publishValve(abrir, method(:onCloud));
    }

    function refresh() as Void {
        if (_client.isBusy()) {
            return;
        }
        _confirming = false;
        _message = _atualizando;
        WatchUi.requestUpdate();
        _client.refresh(method(:onCloud));
    }

    function onCloud(result as Dictionary) as Void {
        var pending = _pendingAbrir;
        _pendingAbrir = null;
        _nivel = result[:nivel];
        _valvula = result[:valvula];
        _cached = result[:cached] == true;
        var error = result[:error];
        if (!(error instanceof Number) || error == CloudClient.ERR_NONE) {
            _message = "";
        } else {
            _message = messageFor(error);
        }
        if (pending != null && !_client.isBusy()) {
            _message = _enviando;
            _client.publishValve(pending, method(:onCloud));
        }
        WatchUi.requestUpdate();
    }

    function messageFor(error as Number) as String {
        if (error == CloudClient.ERR_PHONE) {
            return _semTelefone;
        }
        if (error == CloudClient.ERR_NETWORK) {
            return _semRede;
        }
        if (error == CloudClient.ERR_AUTH) {
            return _acessoNegado;
        }
        if (error == CloudClient.ERR_NOT_FOUND) {
            return _naoEncontrado;
        }
        if (error == CloudClient.ERR_PROPERTIES) {
            return _variavelAusente;
        }
        if (error == CloudClient.ERR_PARSE) {
            return _respostaInvalida;
        }
        if (error == CloudClient.ERR_UNCONFIRMED) {
            return _semConfirmacao;
        }
        if (error == CloudClient.ERR_UNCHANGED) {
            return _naoMudou;
        }
        return _falhaHttp;
    }

    function isConfirming() as Boolean {
        return _confirming;
    }

    function isBusy() as Boolean {
        return _client.isBusy();
    }

    function isPublishing() as Boolean {
        return _client.isPublishing();
    }
}

(:glance)
class CaixaGlanceView extends WatchUi.GlanceView {
    function initialize() {
        GlanceView.initialize();
    }

    function onUpdate(dc as Graphics.Dc) as Void {
        var width = dc.getWidth();
        var height = dc.getHeight();
        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_BLACK);
        dc.clear();
        var nivel = Storage.getValue("nivel");
        var label = "--%";
        if (nivel instanceof Number) {
            label = nivel.toString() + "%";
        } else if (nivel instanceof Float || nivel instanceof Long) {
            label = nivel.toNumber().toString() + "%";
        }
        dc.drawText(width / 2, height / 2, Graphics.FONT_MEDIUM, label, Graphics.TEXT_JUSTIFY_CENTER | Graphics.TEXT_JUSTIFY_VCENTER);
    }
}
