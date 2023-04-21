# gtsam
cd /tmp
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.1.1
mkdir build
cd build
cmake  \
	-DGTSAM_USE_SYSTEM_EIGEN=ON \
	-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
	..
make -j "$(nproc)"
make install

# ceres-solver
cd /tmp
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j "$(nproc)"
make install

cd /tmp
rm -rf *
