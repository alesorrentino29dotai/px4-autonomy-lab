# PX4 — Appunti per il colloquio

## Fase 1 — Setup & primo volo (SITL)

1. **SITL** (Software-In-The-Loop): il firmware PX4 vero gira come processo Linux; Gazebo simula fisica e sensori. Stesso codice del volo reale, quindi i comportamenti (arming, failsafe) sono fedeli.
2. **Convenzione porte MAVLink**: 14550/udp → GCS (QGroundControl), 14540/udp → "onboard"/API (MAVSDK, companion computer). PX4 apre più istanze mavlink con profili diversi (Normal, Onboard, Gimbal).
3. È **PX4 che inizia** lo stream UDP verso la GCS: in Docker serve `--network host` (il port mapping `-p` gestisce solo traffico in ingresso).
4. **Preflight / health checks**: `commander takeoff` è stato rifiutato con "Arming denied: Resolve system health failures first" finché nessuna GCS aveva mai mandato HEARTBEAT — il check "No connection to the GCS" blocca l'arming. Inviare heartbeat (anche da pymavlink, `source_system=255`) lo sblocca.
5. **HEARTBEAT** è il messaggio MAVLink fondamentale: discovery, stato, flight mode (custom_mode); ogni peer lo manda a ~1 Hz.
6. **commander** è il modulo PX4 che possiede la state machine di arming/flight mode; `pxh>` è la shell PX4.
7. Dopo il takeoff il drone passa in **HOLD (Loiter)**: mantiene posizione e quota in autonomia (serve GPS/EKF valido).
8. **EKF2** fonde IMU+GPS+baro+mag: i check "global position ok / home position ok" di MAVSDK derivano dalla validità della stima EKF; la home viene fissata all'arming ed è la destinazione dell'RTL.
9. MAVSDK moderno usa `udpin://0.0.0.0:14540` (ascolta) — `udp://` è deprecato.
10. Log di volo PX4 = file **.ulg** (ULog) scritti dal modulo `logger` — analizzabili con Flight Review.

## M1 — Telemetria

1. La telemetria PX4 arriva come **stream MAVLink a rate fissi** configurati per-istanza dal modulo `mavlink` (vedi `mavlink status` in pxh>): position ~50 Hz in SITL, battery ~1 Hz.
2. **GLOBAL_POSITION_INT** usa interi scalati: lat/lon in gradi ×1e7, quote in mm, velocità NED in cm/s — niente float per robustezza su link radio lenti.
3. Il frame velocità è **NED** (North-East-Down): `vd > 0` significa che stai scendendo. Convenzione aeronautica, opposta all'intuizione "z verso l'alto".
4. **MAVSDK** astrae i messaggi in stream tipizzati per-topic (`telemetry.position()`, `.battery()`, ...): generatori async, uno per topic, da consumare in task asyncio separati.
5. Due quote diverse: **AMSL** (absolute, sul livello del mare) e **relative** (dal punto di home) — l'altitudine di takeoff/atterraggio ragiona in relative.
6. Pattern robusto per logging: task per-stream aggiornano uno snapshot condiviso, un sampler a clock fisso scrive righe rettangolari su CSV (rate diversi → niente buchi).
7. `SYS_STATUS` porta battery e sensor-health bitmask; MAVSDK la espone come `telemetry.health()` (gps_ok, home_ok, armable).
8. Il flight mode viaggia dentro **HEARTBEAT** (`custom_mode`, encoding specifico PX4): non esiste un "messaggio flight mode" dedicato.
