## Compile
- ### 安装OSQP
  ```
  git clone https://github.com/osqp/osqp
  cd osqp
  git checkout release-0.6.3
  mkdir build
  cd build
  cmake -G "Unix Makefiles" ..
  cmake --build .
  sudo cmake --build . --target install
  ```
- ### 编译
  ```
  cd qp_path_generator
  mkdir build
  cd build
  cmake ..
  make
  ./osqp_demo
  ```
