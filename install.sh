#!/bin/bash
set -o errexit # corre do script caso dê algum erro
set -o pipefail # complementa o anterior, veja: http://mywiki.wooledge.org/BashFAQ/105
set -o errtrace  # Segue o erro quando tu chama um comando dentro do outro. Ex: "time find / -iname *opencv*"
set -o nounset # corre do script caso tu tente usar uma variavel não definida ainda

ROBOGEL_CHECKOUT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CTC_FILENAME="ctc-linux64-atom-2.8.5.10" #maybe have the option to pass the ctc as argument --cross-toolchain=~/Downloads/ctc-linux64-atom-2.8.5.10
# eigen-3.4.0.zip
cd "${ROBOGEL_CHECKOUT_DIR}/softwares"
CTC_URL="https://github.com/rinobot-team/SPL-Robogel-Assets/releases/download/v0.1/ctc-linux64-atom-2.8.5.10.zip"
echo "Downloading CTCToolchain..."
wget ${CTC_URL}
unzip -q ${CTC_FILENAME}.zip
rm -f ${CTC_FILENAME}.zip

# back to base dir
cd "${ROBOGEL_CHECKOUT_DIR}"
echo "Relocating QI Toolchain..."
${ROBOGEL_CHECKOUT_DIR}/softwares/${CTC_FILENAME}/yocto-sdk/relocate_qitoolchain.sh # acho que seta a toolchain pra esse lugar aqui. Não sei ao certo

# Eigen para a motion 
EIGEN_ZIP_PATH="${ROBOGEL_CHECKOUT_DIR}/HTWKMotion/eigen/eigen.zip"
echo "Downloading Eigen..."
wget -O ${EIGEN_ZIP_PATH} "https://github.com/rinobot-team/SPL-Robogel-Assets/releases/download/v0.1/eigen-3.4.0.zip"
unzip -q ${EIGEN_ZIP_PATH} -d ${ROBOGEL_CHECKOUT_DIR}/HTWKMotion/eigen/

