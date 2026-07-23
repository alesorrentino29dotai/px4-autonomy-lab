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

## M2 — Arm / Takeoff / Land

1. Ogni comando (arm, takeoff, land) è un **COMMAND_LONG** MAVLink; l'esito torna in **COMMAND_ACK** (ACCEPTED / DENIED / TEMPORARILY_REJECTED) che MAVSDK traduce in `ActionError`.
2. Un arm rifiutato = un **health & arming check** del commander fallito; il motivo viaggia come STATUSTEXT (visibile in QGC o pxh>), non nell'ACK — l'ACK dice solo "DENIED".
3. `MAV_CMD_NAV_TAKEOFF` sale alla quota di takeoff (MAVSDK: `set_takeoff_altitude`, parametro PX4 `MIS_TAKEOFF_ALT`); il completamento va monitorato via telemetria, il comando non "blocca".
4. Lo stato aria/terra viene da **EXTENDED_SYS_STATE** (`MAV_LANDED_STATE`): è così che si sa quando il touchdown è avvenuto davvero.
5. Dopo il land PX4 **disarma automaticamente** (COM_DISARM_LAND, default ~2s a terra) — aspettare `armed == False` è il segnale di fine volo affidabile.
6. Pattern: comandi fire-and-forget + attese basate su telemetria con `asyncio.wait_for` e timeout espliciti — mai fidarsi solo dell'ACK.

## M3 — Missione waypoint & failsafe

1. L'upload missione è un **microservizio MAVLink** (handshake MISSION_COUNT → MISSION_REQUEST_INT → MISSION_ITEM_INT → MISSION_ACK), non un singolo messaggio; lat/lon viaggiano come interi ×1e7.
2. I progressi arrivano da **MISSION_CURRENT / MISSION_ITEM_REACHED**; il raggio di accettazione dei WP è `NAV_ACC_RAD`.
3. **RTL** è governato da `RTL_RETURN_ALT` (quota di rientro), `RTL_DESCEND_ALT` e `RTL_LAND_DELAY`; con `set_return_to_launch_after_mission` la missione termina in AUTO.RTL automaticamente.
4. Soglie batteria (default): `BAT_LOW_THR` 15% → warning; `BAT_CRIT_THR` 7% → azione `COM_LOW_BAT_ACT` (default 3 = return-or-land); `BAT_EMERGEN_THR` 5% → **LAND immediato**.
5. Osservato in SITL: con drain rapidissimo (60 s) si passa da HOLD direttamente a LAND all'emergency — le fasi critical/emergency possono collassare se la batteria crolla più veloce dell'isteresi del commander. `SIM_BAT_MIN_PCT` (default 50) impedisce il drain completo in sim.
6. **Datalink loss**: dichiarato dopo `COM_DL_LOSS_T` secondi senza HEARTBEAT da una GCS; azione = `NAV_DLL_ACT` (0 disabilitato di default!, 2 = Return). Verificato: stop heartbeat → dopo ~10 s AUTO.RTL → atterraggio → disarm.
7. Il failsafe si osserva "da fuori" solo passivamente: un listener che NON manda heartbeat non resetta il timer del datalink — distinzione fondamentale tra ascoltare e essere una GCS.
8. Il flight mode PX4 viaggia in `HEARTBEAT.custom_mode`: main mode nel byte 2, sub-mode nel byte 3 (AUTO=4, sub RTL=5, LAND=6).

## M4 — Offboard control

1. **OFFBOARD** è la modalità in cui un computer esterno (companion) comanda il volo via setpoint continui — il paradigma del controllo da ROS/MAVSDK.
2. Regola d'oro: PX4 accetta il passaggio a OFFBOARD **solo se un flusso di setpoint è già attivo** (>2 Hz); per questo si manda un setpoint prima di `offboard.start()`.
3. Ogni setpoint è un **SET_POSITION_TARGET_LOCAL_NED** con `type_mask` che dice quali campi valgono (posizione, velocità, accelerazione, yaw) — qui velocità+yaw.
4. Frame **NED locale**: origine al punto di arming, z positiva verso il basso; `vd=0` mantiene la quota, il cerchio è solo la rotazione continua del vettore (vn, ve).
5. Se il flusso di setpoint si interrompe scatta l'**offboard-loss failsafe** (`COM_OF_LOSS_T`, azione `COM_OBL_RC_ACT`) — stessa filosofia del datalink loss di M3.
6. Traiettorie "a velocità": semplici e robuste ma in anello aperto sulla posizione — la deriva si accumula (il quadrato non si richiude perfettamente). Per M5 la correzione arriverà in anello chiuso dalla visione.
7. Lo yaw è comandabile indipendentemente dalla direzione di volo (nose sul tangente nel cerchio) — un quadricottero è olonomo nel piano.

## M5 — Precision landing su ArUco

1. Architettura a due nodi come su un drone vero: **percezione vicino al sensore** (nodo CV nel container, via gz-transport), **guida altrove** (MAVSDK sull'host), contratto minimale in mezzo (JSON/UDP con angoli, non pixel).
2. **Angoli, non pixel**, attraversano l'interfaccia: `ang = atan((u-cx)/fx)`; moltiplicati per la quota danno l'offset metrico a qualunque risoluzione — stesso principio del messaggio MAVLink `LANDING_TARGET`.
3. Il marker PX4 del mondo `aruco` è **DICT_4X4_50 id 0**, 0.5 m; il dizionario si può auto-scoprire provando i più comuni sul primo frame.
4. Mapping camera downward (pitch +90°): image-right → est, image-down → −nord (a yaw 0). Sbagliare un segno = divergenza immediata: verificarlo è la prima cosa da fare.
5. Legge di controllo: P laterale sull'offset metrico (`Kp=0.6`), e **discesa solo se centrato** (soglia ∝ quota) — l'imbuto si auto-corregge, ogni deriva mette in pausa la discesa.
6. Endgame: sotto 1 m il marker esce dal FOV → handover a `action.land()`. Su hardware reale: marker annidato grande+piccolo per coprire tutte le quote.
7. Marker perso >1 s → hold, mai scendere alla cieca. Risultato in SITL: errore di touchdown **2–3 cm** partendo da 5 m di offset a 8 m di quota.
8. Alternativa "di produzione": mandare `LANDING_TARGET` al modulo precision-landing nativo di PX4 (parametri `PLD_*`), che gestisce search pattern e failsafe internamente.

## M6 — MAVSDK C++

1. MAVSDK C++ distribuito come `.deb` prebuilt: senza sudo si estrae con `dpkg -x` in un prefix locale e si compila con `CMAKE_PREFIX_PATH` + RPATH — i symlink `libmavsdk.so.3` vanno creati a mano (li farebbe ldconfig).
2. API C++ v3: `Mavsdk` vuole una `Configuration{ComponentType::GroundStation}` esplicita; `first_autopilot(timeout)` sostituisce la vecchia danza di discovery.
3. Python è async-first (await/generatori), il C++ offre chiamate bloccanti + subscription a callback: stesso protocollo MAVLink sotto, diversa ergonomia.
4. `telemetry.health_all_ok()` riassume in una chiamata il gate EKF che in Python richiedeva l'iterazione dello stream health.
5. Gli enum result (`Action::Result`) stampabili con operator<< danno diagnostica arm/takeoff equivalente agli `ActionError` Python.
