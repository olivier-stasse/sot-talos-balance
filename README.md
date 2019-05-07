# sot-talos-balance

Coordination project for the control of the balance of Talos.

## Installation procedure
Assuming you have all the dependencies correctly installed inside `/opt/openrobots`,
the package is installed as follows.
The detailed explanation of the commands is available in the documentation.

```
git clone --recursive git@gepgitlab.laas.fr:loco-3d/sot-talos-balance.git
cd sot-talos-balance
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots ..
make -j4
make install
```

