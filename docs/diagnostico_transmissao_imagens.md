# Procedimento de Diagnóstico — Impacto da Transmissão de Imagens

**Drone:** Pixhawk 6X + PX4 1.15.4 + Jetson Orin Nano  
**Data:** 17/05/2026  **Executado por:** Ângelo

---

## Contexto e hipótese

O FSM envia `offboard_control_mode` e `trajectory_setpoint` ao PX4 a **20 Hz** via timer de 50 ms. O PX4 exige esses sinais a ≥ 2 Hz; se não chegarem, ele sai do modo Offboard e entra em failsafe — que pode incluir subida automática de segurança ("Icarus").

**Hipótese:** processamento de imagens (OpenCV + OCR) sobrecarrega a CPU do Jetson → o timer de 50 ms atrasa → sinais chegam abaixo de 2 Hz ao PX4 → PX4 sai do Offboard → subida descontrolada.

**Métrica-chave de toda a sessão:**
```
ros2 topic hz /fmu/in/offboard_control_mode --window 20
```
Deve mostrar **≥ 18 Hz** sob qualquer carga. Abaixo disso é problema.

---

## Preparação: 4 terminais de monitoramento

Antes de qualquer fase, abrir estes terminais e mantê-los rodando o tempo todo.

**Terminal M1 — Taxa de controle (critério go/no-go):**
```bash
source ~/evtol/dev/install/setup.bash
watch -n 1 'ros2 topic hz /fmu/in/offboard_control_mode --window 20 2>&1 | head -5'
```

**Terminal M2 — CPU/RAM/Temperatura do Jetson:**
```bash
tegrastats --interval 500
# Observar: CPU [X%@freq,...], RAM X/YMB, tj X°C
```

**Terminal M3 — Modo do PX4:**
```bash
source ~/evtol/dev/install/setup.bash
ros2 topic echo /fmu/out/vehicle_status --field nav_state
# 14 = Offboard  3 = Hold  0 = Manual  qualquer outro = problema
```

**Terminal M4 — Bandwidth de imagens (usar quando necessário):**
```bash
source ~/evtol/dev/install/setup.bash
ros2 topic bw /vertical_camera/compressed
# Trocar pelo tópico que quiser medir
```

---

## FASE A — Software em terra (Jetson ligado, drone desligado)

> Sem risco físico. Propósito: isolar cada camada de carga e medir o impacto no Hz de controle.

---

### A0 — Baseline: FSM sem nenhum nó de visão

**Setup:**

```bash
# Terminal 1: agente uXRCE-DDS
~/evtol/dev/src/sae2026/scripts/agent.sh
```

```bash
# Terminal 2: FSM sozinho
source ~/evtol/dev/install/setup.bash
ros2 run mission_1_H mission_1_H --ros-args \
  --params-file ~/evtol/dev/install/mission_1_H/share/mission_1_H/config/simulation.yaml
```

**Aguardar 2 minutos. Preencher:**

| Métrica | Valor medido | Esperado |
|---|---|---|
| Hz `/fmu/in/offboard_control_mode` | ___ Hz | 19–21 Hz |
| CPU total (tegrastats) | ___% | < 30% |
| Temperatura Jetson | ___°C | < 60°C |

**Critério de PARE:** Hz < 18 Hz → problema no FSM ou no agente DDS, não nos nós de visão. Investigar antes de continuar.

---

### A1 — Adicionar câmera (publicando, sem assinante de processamento)

Manter Terminal 1 e 2 do A0. Adicionar:

```bash
# Terminal 3: câmera publicando
source ~/evtol/dev/install/setup.bash
ros2 run camera_publisher webcam --ros-args \
  -p video_source:=/dev/video2 \
  -p camera_name:=vertical \
  -p frame_width:=640 \
  -p frame_height:=640 \
  -p publish_rate:=10.0 \
  -p jpeg_quality:=70
```

**Aguardar 1 minuto. Preencher:**

| Métrica | Valor medido | Delta vs A0 |
|---|---|---|
| Hz `/fmu/in/offboard_control_mode` | ___ Hz | ___ |
| CPU total | ___% | +___% |
| Bandwidth `/vertical_camera/compressed` (M4) | ___ kB/s | — |

---

### A2 — Adicionar nó de visão sem debug

Manter Terminais 1, 2 e 3 do A1. Adicionar:

```bash
# Terminal 4: nó de visão, debug desabilitado
source ~/evtol/dev/install/setup.bash
ros2 run RDPformas RDPformas --ros-args \
  --params-file ~/evtol/dev/install/RDPformas/share/RDPformas/config/rdpformas.yaml \
  -p debug_image:=false \
  -p debug_mask:=false
```

**Aguardar 1 minuto. Preencher:**

| Métrica | Valor medido | Delta vs A1 |
|---|---|---|
| Hz `/fmu/in/offboard_control_mode` | ___ Hz | ___ |
| CPU total | ___% | +___% |
| Latência média do tópico de controle | ___ ms | — |

Medir latência:
```bash
ros2 topic delay /fmu/in/offboard_control_mode
```

---

### A3 — Debug images habilitadas (carga máxima)

Matar o nó de visão do A2 (Ctrl+C no Terminal 4) e reiniciar com debug ligado:

```bash
# Terminal 4: nó de visão COM debug
source ~/evtol/dev/install/setup.bash
ros2 run RDPformas RDPformas --ros-args \
  --params-file ~/evtol/dev/install/RDPformas/share/RDPformas/config/rdpformas.yaml \
  -p debug_image:=true \
  -p debug_publish_rate:=5.0 \
  -p debug_max_width:=640
```

**Aguardar 2 minutos. Preencher:**

| Métrica | Valor medido | Limite crítico |
|---|---|---|
| Hz `/fmu/in/offboard_control_mode` | ___ Hz | **≥ 18 Hz** |
| CPU total | ___% | < 90% |
| Temperatura Jetson | ___°C | < 80°C |
| Bandwidth `/bouncing_detection_image/compressed` (M4) | ___ kB/s | — |

**Interpretação do Hz medido:**

| Hz medido | Conclusão |
|---|---|
| ≥ 18 Hz | Debug images não causam problema — sistema saudável |
| 10–18 Hz | Degradação preocupante — mitigação necessária antes de voo |
| < 10 Hz | Problema grave — confirma hipótese de CPU starvation |

---

### A4 — Mitigação: prioridade elevada no FSM

Matar FSM (Ctrl+C no Terminal 2) e reiniciar com `nice -n -10`. Manter debug ligado (A3 continua):

```bash
# Terminal 2: FSM com prioridade elevada
source ~/evtol/dev/install/setup.bash
nice -n -10 ros2 run mission_1_H mission_1_H --ros-args \
  --params-file ~/evtol/dev/install/mission_1_H/share/mission_1_H/config/simulation.yaml
```

**Aguardar 1 minuto. Preencher:**

| Métrica | Valor medido | vs A3 (sem nice) |
|---|---|---|
| Hz `/fmu/in/offboard_control_mode` | ___ Hz | +/- ___ Hz |
| CPU total | ___% | +/-___% |

**Critério de sucesso:** Hz ≥ 18 Hz → `nice -n -10` compensa a carga de visão. Confirma que essa mitigação deve ser mantida em todos os launches de voo.

---

## FASE B — Drone ligado em terra (sem decolar)

> **Pré-requisito:** Fase A completa com Hz ≥ 18 Hz em A3 ou A4.  
> **Segurança obrigatória:** remover hélices OU prender o drone fisicamente ao chão antes de ligar.

---

### B1 — Estabilidade do modo Offboard em terra

Usar o launch completo normal:

```bash
source ~/evtol/dev/install/setup.bash
ros2 launch mission_1_H flight.launch.py
```

Via QGroundControl:
1. Armar o drone
2. Colocar em modo Offboard

Observar Terminal M3 (`nav_state`).

**Aguardar 5 minutos com todos os nós CV rodando. Preencher:**

| Tempo | nav_state | Evento |
|---|---|---|
| 0:00 | | Início Offboard |
| 1:00 | | |
| 2:00 | | |
| 3:00 | | |
| 4:00 | | |
| 5:00 | | |

> `nav_state = 14` o tempo todo = **PASS** ✓  
> Qualquer outra transição = **FAIL** ✗ — não prosseguir para voo.

---

### B2 — Gravação diagnóstica completa

Com drone armado no Offboard e todos os nós rodando, iniciar gravação:

```bash
ros2 bag record \
  /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint \
  /fmu/out/vehicle_status \
  /fmu/out/vehicle_local_position \
  /telemetry/system_health \
  -o ~/evtol/mission_logs/diagnostico_$(date +%Y%m%d_%H%M%S)
```

**Gravar por 5 minutos.** Esse bag é a evidência para análise pós-teste.

Após gravar, checar taxa real no bag:
```bash
ros2 bag info ~/evtol/mission_logs/diagnostico_*/
```

---

## FASE C — Voos incrementais

> **Pré-requisito OBRIGATÓRIO:** B1 aprovado (`nav_state = 14` estável durante todo o B1).  
> **Local:** área aberta, sem obstáculos. Operador pronto para assumir controle manual.

---

### C1 — Voo de referência (sem nenhum nó de visão)

**Objetivo:** confirmar que hardware e PX4 estão saudáveis sem carga de visão.

```bash
# Comentar bouncing_cv_node no flight.launch.py antes de lançar
source ~/evtol/dev/install/setup.bash
ros2 launch mission_1_H flight.launch.py
```

**Hover por 30 segundos. Preencher:**

| Comportamento observado | Sim / Não |
|---|---|
| Subida controlada até altitude alvo | |
| Hover estável (drift < 0,5 m em 30 s) | |
| **Icarus (subida descontrolada)** | |

> **Se Icarus acontecer aqui:** o problema é de hardware/EKF/sensor — não de visão computacional. Parar os testes desta série e investigar EKF2 e sensores.

---

### C2 — Voo com CV, debug desabilitado

```bash
source ~/evtol/dev/install/setup.bash
ros2 launch mission_1_H flight.launch.py
# rdpformas.yaml já tem debug_image: false
```

**Hover por 60 segundos. Preencher:**

| Comportamento | Observado |
|---|---|
| Hover estável | Sim / Não |
| Drift horizontal em 60 s | ___ m |
| **Icarus** | Sim / **Não** |
| Hz de controle (analisar bag depois) | ___ Hz |
| Temperatura Jetson ao final | ___°C |

---

### C3 — Voo com debug habilitado

Só realizar se C2 aprovado. Alterar YAML antes de lançar:

```bash
# Editar temporariamente:
# debug_image: true, debug_publish_rate: 2.0
source ~/evtol/dev/install/setup.bash
ros2 launch mission_1_H flight.launch.py
```

**Regra de abort:** se drone subir > 1 m além da altitude comandada → assumir controle manual imediatamente.

| Comportamento | Observado |
|---|---|
| Hover estável | Sim / Não |
| **Icarus** | Sim / **Não** |
| Hz de controle (bag) | ___ Hz |

---

## Tabela de resumo — preencher ao final da sessão

| Teste | Hz medido | CPU% | Temp°C | Resultado |
|---|---|---|---|---|
| A0 Baseline | | | | PASS / FAIL |
| A1 +Câmera | | | | PASS / FAIL |
| A2 +CV sem debug | | | | PASS / FAIL |
| A3 +Debug ligado | | | | PASS / FAIL |
| A4 +nice -n -10 | | | | PASS / FAIL |
| B1 Offboard terra | | | | PASS / FAIL |
| B2 Gravação | | | | Gravado / — |
| C1 Voo sem CV | | | | PASS / FAIL |
| C2 Voo sem debug | | | | PASS / FAIL |
| C3 Voo com debug | | | | PASS / FAIL |

---

## Árvore de decisão pós-testes

```
A0 Hz < 18? ─── SIM ──→ Problema no FSM ou agente DDS.
     │                   Verificar agent.sh e timer do FSM (50ms).
    NÃO
     │
A3 Hz < 18? ─── SIM ──→ Debug images sobrecarregam CPU.
     │                   Ação: manter debug_image: false em voo de competição.
     │
     │           A4 Hz ≥ 18? ─── SIM ──→ nice -n -10 resolve. Manter nos launches.
     │                    └── NÃO ──→ Reduzir processing_frequency ou desativar CV.
    NÃO
     │
C1 Icarus? ──── SIM ──→ Problema de hardware/EKF. Investigar EKF2_HGT_REF e sensores.
     │
    NÃO
     │
C2 Icarus? ──── SIM ──→ CV sem debug ainda sobrecarrega em voo.
     │                   Reduzir processing_frequency no YAML do detector.
    NÃO
     │
     ▼
Sistema aprovado para uso em competição.
Debug pode ser habilitado com cautela (verificar C3).
```

---

## Notas e observações do teste

```
_________________________________________________________________________

_________________________________________________________________________

_________________________________________________________________________

_________________________________________________________________________

_________________________________________________________________________
```
