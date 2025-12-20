#include <stdio.h>
#include <string>

constexpr unsigned int float_offset = 1;

struct VLAInput {
  float dx;
  float dy;
  float dz;
  float roll;
  float pitch;
  float yaw;
  float gripper_strength;
  float term;
};

void parse_next(std::string *vla_input_ptr, char token, int *from_index, float *ret) {
  int id = vla_input_ptr->find(token, *from_index);
  *ret = std::stof(vla_input_ptr->substr(*from_index, id));
  *from_index = id + 1;
}

VLAInput parse_input(std::string vla_input_str) {
  int id = 0;
  char tok = ',';
  VLAInput ret;
  for (int i=0; i<8; i++) {
    parse_next(&vla_input_str, tok, &id, (float *)&ret + i);
  }
  return ret;
}

int main() {
  std::string vla_input_str = "0.2, 0.3, 0.5, 20, 60, 40, -0.8, 0";
  VLAInput x = parse_input(vla_input_str);
  for (int i=0, offset=0; i<8; i++, offset+=float_offset) {
    printf("%f\n", *((float *)&x + offset));
  }
  printf("%f\n", x.roll);
  return 0;
}
