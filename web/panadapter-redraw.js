(function (root) {
    "use strict";

    function isPitchField(id) {
        return id === "PITCH" || id === "FTX_RX_PITCH" || id === "TX_PITCH";
    }

    function create(redraw) {
        let latestSpectrum = null;

        return {
            spectrumUpdated(update) {
                latestSpectrum = update;
            },

            fieldUpdated(id) {
                if (isPitchField(id) && latestSpectrum !== null)
                    redraw(latestSpectrum);
            }
        };
    }

    const api = { create, isPitchField };
    root.WebPanadapterRedraw = api;
    if (typeof module === "object" && module.exports)
        module.exports = api;
}(typeof globalThis === "object" ? globalThis : this));
