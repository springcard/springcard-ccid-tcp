#!/bin/sh
mkdir src/CCID/
cd src/CCID/
git clone https://github.com/LudovicRousseau/CCID.git
cd ..
git clone https://github.com/LudovicRousseau/PCSC.git
cd ..
make

