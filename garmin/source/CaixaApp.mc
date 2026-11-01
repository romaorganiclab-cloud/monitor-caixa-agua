import Toybox.Application;
import Toybox.WatchUi;

class CaixaApp extends Application.AppBase {
    function initialize() {
        AppBase.initialize();
    }

    function getInitialView() {
        var view = new CaixaView();
        return [view, new CaixaDelegate(view)];
    }

    function getGlanceView() {
        return [new CaixaGlanceView()];
    }
}
