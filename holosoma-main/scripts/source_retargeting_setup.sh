# Detect script directory (works in both bash and zsh)
if [ -n "${BASH_SOURCE[0]}" ]; then
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
elif [ -n "${ZSH_VERSION}" ]; then
    SCRIPT_DIR=$( cd -- "$( dirname -- "${(%):-%x}" )" &> /dev/null && pwd )
fi

CONDA_ENV_NAME=${CONDA_ENV_NAME:-hsretargeting}
echo "conda environment name is set to: $CONDA_ENV_NAME"

source "${SCRIPT_DIR}/source_common.sh"
unalias python python3 pip pip3 tensorboard 2>/dev/null || true

if [ -f "${CONDA_ROOT}/bin/activate" ]; then
    source "${CONDA_ROOT}/bin/activate" "$CONDA_ENV_NAME"
elif command -v conda >/dev/null 2>&1 && conda env list | awk '{print $1}' | grep -qx "$CONDA_ENV_NAME"; then
    CONDA_BASE=$(conda info --base)
    if [ -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]; then
        source "${CONDA_BASE}/etc/profile.d/conda.sh"
    fi
    conda activate "$CONDA_ENV_NAME"
else
    echo "ERROR: Retargeting conda environment '$CONDA_ENV_NAME' was not found." >&2
    echo "Expected activate script: ${CONDA_ROOT}/bin/activate" >&2
    echo "Run this first from the repo root:" >&2
    echo "  bash scripts/setup_retargeting.sh" >&2
    return 1 2>/dev/null || exit 1
fi
hash -r 2>/dev/null || true
