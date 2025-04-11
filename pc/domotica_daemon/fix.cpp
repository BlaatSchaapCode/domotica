/*
 * fix.cpp
 *
 *  Created on: 11 apr. 2025
 *      Author: andre
 */

#include "fix.hpp"

fix operator+(const float &lhs, const fix &rhs) {
  fix result;
  result.value = (lhs * 256.0f) + rhs.value;
  return result;
}
