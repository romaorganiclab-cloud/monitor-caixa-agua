import Toybox.Lang;
import Toybox.WatchUi;

// InputDelegate, not BehaviorDelegate. On the Venu 3 a BehaviorDelegate
// turns every screen tap into onSelect, the same call as the upper button,
// and onSelect has no coordinates. The label tap was refreshing the level.
class CaixaDelegate extends WatchUi.InputDelegate {
    var _view as CaixaView;

    function initialize(view as CaixaView) {
        InputDelegate.initialize();
        _view = view;
    }

    function onTap(clickEvent as WatchUi.ClickEvent) as Boolean {
        var coords = clickEvent.getCoordinates();
        return _view.handleTap(asNumber(coords[0]), asNumber(coords[1]));
    }

    function onKey(keyEvent as WatchUi.KeyEvent) as Boolean {
        var key = keyEvent.getKey();
        if (key == WatchUi.KEY_ENTER) {
            if (!_view.isConfirming() && !_view.isPublishing()) {
                _view.refresh();
            }
            return true;
        }
        if (key == WatchUi.KEY_ESC) {
            WatchUi.popView(WatchUi.SLIDE_RIGHT);
            return true;
        }
        return false;
    }

    function onSwipe(swipeEvent as WatchUi.SwipeEvent) as Boolean {
        if (swipeEvent.getDirection() == WatchUi.SWIPE_RIGHT) {
            WatchUi.popView(WatchUi.SLIDE_RIGHT);
            return true;
        }
        return false;
    }

    function asNumber(value as Object) as Number {
        if (value instanceof Float) {
            return value.toNumber();
        }
        if (value instanceof Long) {
            return value.toNumber();
        }
        return value as Number;
    }
}
