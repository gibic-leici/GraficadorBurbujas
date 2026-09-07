# GraficadorBurbujas

Graficador y monitor de señales ADC por puerto COM (CDC USB / Web Serial API) para el sistema de medición de burbujas (GIBIC - LEICI).

## Características
- Adquisición en tiempo real de 2 canales ADC (`uint16_t`).
- 100 muestras por canal por paquete (200 muestras totales por paquete).
- Visualización interactiva con canvas.
- Configuración de escala temporal, voltajes y disparador (trigger).
- Exportación de datos.

## Uso
Abrir `index.html` en un navegador compatible con la **Web Serial API** (Google Chrome, Microsoft Edge, Opera).
