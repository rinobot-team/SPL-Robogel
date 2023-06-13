export ROBOGEL_CHECKOUT_DIR="$(realpath .)"
export CTC_FILENAME="ctc-linux64-atom-2.8.5.10"
eigen-3.4.0.zip
cd "${ROBOGEL_CHECKOUT_DIR}/softwares
# https://github.com/rinobot-team/SPL-Robogel-Assets/releases/download/v0.1/ctc-linux64-atom-2.8.5.10.zip
unzip ${CTC_FILENAME}.zip || $(echo "Failed to unzip ctc, you need to put a ${CTC_FILENAME}.zip in the softwares folder" ; exit 1)

# back to base dir
cd "${ROBOGEL_CHECKOUT_DIR}
# unzip 
./${ROBOGEL_CHECKOUT_DIR}/softwares/${CTC_FILENAME}/yocto-sdk/relocate_qitoolchain.sh || $(echo "couldnt relocate qitoolchain" ; exit 1)

