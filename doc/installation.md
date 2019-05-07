# Installation

1. Clone the git repository:
```
git clone --recursive git@gepgitlab.laas.fr:loco-3d/sot-talos-balance.git
cd sot-talos-balance
```

2. If you need it, switch to the devel branch
```
git checkout devel
```

3. Create the build directory and move there
```
mkdir build
cd build
```

4. Run cmake
```
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots ..
```

5. Build the package
```
make -j4
```

6. Install the package
```
make install
```

