#!/bin/bash


mkdir auto_install
cd auto_install

# installation of IPOPT (optimization library)
svn co https://projects.coin-or.org/svn/Ipopt/stable/3.11 CoinIpopt3_11    
cd CoinIpopt3_11/ThirdParty/ASL
./get.ASL	# wget http://netlib.sandia.gov/cgi-bin/netlib/netlibfiles.tar?filename=netlib/ampl/solvers 

cd ../Blas
./get.Blas

cd ../Lapack
./get.Lapack

cd ../Metis-
./get.Metis

cd ../Mumps
./get.Mumps


cd ../../
pwd
./configure --prefix=/usr/local/  CXXFLAGS=-fopenmp
make
make test
sudo make install
cd ../

