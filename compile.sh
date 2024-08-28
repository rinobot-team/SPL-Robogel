#!/bin/bash
set -o errexit # corre do script caso dê algum erro
set -o pipefail # complementa o anterior, veja: http://mywiki.wooledge.org/BashFAQ/105
set -o errtrace  # Segue o erro quando tu chama um comando dentro do outro. Ex: "time find / -iname *opencv*"
set -o nounset # corre do script caso tu tente usar uma variavel não definida ainda

ROBOGEL_CHECKOUT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
mkdir -p "${ROBOGEL_CHECKOUT_DIR}/build"
cd "${ROBOGEL_CHECKOUT_DIR}/build"
cmake .. -DCMAKE_TOOLCHAIN_FILE="../cross-config.cmake"
make

