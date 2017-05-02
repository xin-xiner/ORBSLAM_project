#pragma once
#include <iostream>

#define print_value(x) std::cout<<#x<<": "<<x<<std::endl;
#define print_vect(x) std::cout<<#x<<std::endl<<x.t()<<std::endl;
#define print_mat(x) std::cout<<#x<<std::endl<<x<<std::endl;
#define print_vector(x) std::cout<<#x<<std::endl;for(int i = 0;i<x.size();i++){std::cout<<x[i]<<std::endl;}std::cout<<std::endl<<std::endl