#!/usr/bin/env bash
# =============================================================================
# Template: simulate.sh — sobe o PX4 SITL + Gazebo para um mundo.
# =============================================================================
#
#   ./scripts/simulate.sh <mundo>
#
# Copie para o scripts/ da sua competição e preencha o bloco `case` com os
# mundos, modelos e poses iniciais dela. É o ÚNICO arquivo dos três templates
# que exige customização de verdade.
# =============================================================================
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# <ws>/src/<competicao>/scripts/  ->  <ws>
ws_root="$(cd "$script_dir/../../.." && pwd)"

# Carrega a distro do perfil desta máquina + o install/ do workspace.
# Nunca escreva `source /opt/ros/humble/setup.bash` aqui: o time voa com
# Jetson (Humble) e Raspberry Pi (Jazzy), e o mesmo script tem que servir aos
# dois. Veja docs/ARCHITECTURE.md, "O Contrato de Ambiente".
source "$ws_root/scripts/ros_env.sh"

# MODELOS: resolvidos pelo Gazebo via GZ_SIM_RESOURCE_PATH, e nao precisam
# estar dentro da arvore do PX4. O proprio PX4 acrescenta a arvore dele ao
# final desta variavel (veja gz_env.sh), entao o que exportamos aqui vem
# ANTES e tem prioridade.
#
# A ordem importa e ja causou bug: quando a arvore do PX4 vinha primeiro, uma
# copia antiga de modelo escondia a deste repositorio, e a versao que rodava
# nao era a versionada. Com o repo da equipe na frente, ele e a fonte da
# verdade e pode ate sobrescrever um modelo do proprio PX4.
#
# (As variaveis GAZEBO_* sao do Gazebo classico e nao funcionam aqui.)
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-gazebo-models/models:${GZ_SIM_RESOURCE_PATH:-}"

cd "$HOME/PX4-Autopilot"

PX4_SYS_AUTOSTART=4001

case "${1:-}" in
    sae1)
        PX4_GZ_WORLD=sae1_26
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_sae
        ;;
    sae2)
        PX4_GZ_WORLD=sae2_26
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.50, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_dual_cam
        ;;
    sae3)
        PX4_GZ_WORLD=sae3_26
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_sae
        ;;
    default)
        PX4_GZ_WORLD=default
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500
        ;;
    *)
        echo "Mundo desconhecido: '${1:-}'" >&2
        echo "Uso: $0 <mundo>" >&2
        echo "Disponíveis: sae1, sae2, sae3, default" >&2
        exit 1
        ;;
esac

# ---------------------------------------------------------------------------
# Confere que o mundo e o modelo existem ANTES de chamar o PX4.
#
# Sem isto, o PX4 falha com uma mensagem que aponta para o lugar errado:
#
#     Unable to find or download file
#     ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH
#     ERROR [init] gz_bridge failed to start and spawn model
#
# ...o que faz todo mundo ir mexer no GZ_SIM_RESOURCE_PATH, quando a causa
# quase sempre e outra: o .sdf existe em ~/PX4-gazebo-models mas nao foi
# symlinkado para dentro do PX4. Os symlinks sao feitos uma vez; arquivo novo
# adicionado depois nao ganha symlink sozinho.
# ---------------------------------------------------------------------------
px4_gz="$HOME/PX4-Autopilot/Tools/simulation/gz"
faltando=0

# MUNDOS: o PX4 monta um caminho ABSOLUTO na arvore dele e passa ao gz sim
# (px4-rc.simulator: `gz sim -s "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf"`).
# Diferente dos modelos, aqui o arquivo precisa mesmo aparecer la dentro.
# Criamos o link do mundo que vamos lancar, e so dele -- assim ninguem
# precisa lembrar de refazer symlink quando um mundo novo entra no repo.
if [[ ! -e "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" \
   && -e "$HOME/PX4-gazebo-models/worlds/$PX4_GZ_WORLD.sdf" ]]; then
    echo "Criando link para o mundo '$PX4_GZ_WORLD' na arvore do PX4"
    ln -sf "$HOME/PX4-gazebo-models/worlds/$PX4_GZ_WORLD.sdf" "$px4_gz/worlds/"
fi

if [[ ! -e "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" ]]; then
    echo "ERRO: mundo '$PX4_GZ_WORLD.sdf' nao encontrado em" >&2
    echo "      $px4_gz/worlds/" >&2
    echo "      -> nao existe nem em ~/PX4-gazebo-models. Crie o .sdf la." >&2
    faltando=1
fi

# MODELOS: basta existir no repositorio da equipe ou na arvore do PX4 --
# os dois estao no GZ_SIM_RESOURCE_PATH. Nao ha symlink de modelo.
if [[ ! -e "$HOME/PX4-gazebo-models/models/$PX4_SIM_MODEL" \
   && ! -e "$px4_gz/models/$PX4_SIM_MODEL" ]]; then
    echo "ERRO: modelo '$PX4_SIM_MODEL' nao encontrado." >&2
    echo "      Procurado em ~/PX4-gazebo-models/models/ e em" >&2
    echo "      $px4_gz/models/" >&2
    echo "      -> crie o modelo em ~/PX4-gazebo-models/models/." >&2
    faltando=1
fi

if (( faltando )); then
    echo >&2
    echo "Veja docs/gazebo_models_setup.md." >&2
    exit 1
fi

PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
PX4_GZ_MODEL_POSE=$PX4_GZ_MODEL_POSE \
PX4_GZ_WORLD=$PX4_GZ_WORLD \
PX4_SIM_MODEL=$PX4_SIM_MODEL \
./build/px4_sitl_default/bin/px4
