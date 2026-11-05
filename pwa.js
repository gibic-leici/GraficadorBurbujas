if ("serviceWorker" in navigator && window.isSecureContext) {
    window.addEventListener("load", () => {
        navigator.serviceWorker.register("./service-worker.js")
            .catch((error) => console.error("No se pudo registrar el modo sin conexión:", error));
    });
}
