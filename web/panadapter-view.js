(function (root) {
    "use strict";

    function clamp(view) {
        view.zoom = Math.max(1, Math.min(view.maxZoom, view.zoom));
        const limit = 0.5 - 0.5 / view.zoom;
        view.center = Math.max(-limit, Math.min(limit, view.center));
        return view;
    }

    function create(baseSpanHz, maxZoom) {
        return {
            baseSpanHz: baseSpanHz,
            maxZoom: maxZoom,
            zoom: 1,
            center: 0
        };
    }

    function zoomAt(view, factor, position) {
        const oldZoom = view.zoom;
        view.zoom *= factor;
        clamp(view);
        view.center += (position - 0.5) *
            (1 / oldZoom - 1 / view.zoom);
        return clamp(view);
    }

    function pan(view, visibleFraction) {
        view.center += visibleFraction / view.zoom;
        return clamp(view);
    }

    function reset(view) {
        view.zoom = 1;
        view.center = 0;
        return view;
    }

    function spanHz(view) {
        return Math.max(1, Math.round(view.baseSpanHz / view.zoom));
    }

    function tunedX(view, width) {
        return width * (0.5 - view.center * view.zoom);
    }

    function sourceBin(view, x, width, count) {
        if (count <= 1 || width <= 1)
            return 0;
        const fullPosition = 0.5 + view.center +
            (x / (width - 1) - 0.5) / view.zoom;
        return Math.max(0, Math.min(count - 1,
            Math.round((1 - fullPosition) * (count - 1))));
    }

    const api = { create, zoomAt, pan, reset, spanHz, tunedX, sourceBin };
    root.PanadapterView = api;
    if (typeof module === "object" && module.exports)
        module.exports = api;
}(typeof globalThis === "object" ? globalThis : this));
