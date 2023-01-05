cd /tmp
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.1.1
mkdir build
cd build
cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j "$(nproc)"
make install
cd /tmp
rm -rf *
