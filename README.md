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
Abrir `index.html` en un navegador compatible con la **Web Serial API** (Google Chrome, Microsoft Edge, Opera).
