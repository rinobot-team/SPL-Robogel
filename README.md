# SPL-Robogel
Rinobot SPL Code Release

LoLA Docs: http://doc.aldebaran.com/2-8/naoqi/lola/lola.html

Para compilar, coloque uma ctc-2.8.5.10 na pasta ../softwares
de o relocate_qitoolchain na ctc (o comando esta descrito no arquivo ../install.sh)

mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../cross-config.cmake
make