# GraficadorBurbujas

Graficador y monitor de señales ADC por puerto COM (CDC USB / Web Serial API) para el sistema de medición de burbujas (GIBIC - LEICI).

## Características
- Adquisición en tiempo real de 2 canales ADC (`int16_t`).
- 256 muestras por canal por paquete (`MAX_BUFFER_LEN = 256`).
- Metadatos en cabecera: `samples_per_period` y `clock_prescaler` (`uint16_t`), más contador de secuencia `seq` (`uint32_t`).
- Tamaño de paquete: 1036 bytes (`4B header + 2B samples_per_period + 2B clock_prescaler + 4B seq + 512B ch0 + 512B ch1`).
- Visualización interactiva en canvas con autoescala y decimación configurable.
- Exportación y registro a CSV.

## Uso
Abrir `index.html` desde un contexto seguro (por ejemplo, HTTPS) en Chrome, Edge u Opera.

- En PC, usar **Conectar por serie** (Web Serial API).
- En Android, conectar el dispositivo mediante un adaptador OTG y usar **Conectar por USB**. Esta opción usa el polyfill local de `vendor/web-serial-polyfill.js` sobre WebUSB. No requiere acceso a una CDN; el navegador debe permitir WebUSB.
- Para instalarla en Android, abrir la aplicación desde Chrome con una dirección HTTPS y elegir **Instalar aplicación** en el menú del navegador. La primera visita debe realizarse con conexión a internet; después, la interfaz puede abrirse sin conexión. La conexión USB continúa requiriendo un contexto seguro y acceso al dispositivo.

El polyfill está basado en `web-serial-polyfill` 1.0.15 (Copyright 2019 Google LLC), bajo Apache License 2.0. Se conserva la licencia en `vendor/LICENSE.web-serial-polyfill`; el código local incluye el filtro de lecturas USB vacías requerido por algunos dispositivos.
