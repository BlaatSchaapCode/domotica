/*
 * fix.hpp
 *
 *  Created on: 11 apr. 2025
 *      Author: andre
 */

#ifndef FIX_HPP_
#define FIX_HPP_

#include <cstdint>

class fix {
	public:
	// This seems to work so far
	fix   operator=(const float& rhs) {
		value = rhs * 256.0f;
		return *this;
		}

	operator float()  {return (float)this->value/256.0f; };


  fix operator+ (const fix &rhs) const {
	fix result;
	result.value = this->value + rhs.value;
	return result;
  }

  fix operator+ (const float &rhs) const {
	fix result;
	result.value = value + (rhs * 256.0f);
	return result;
  }

	friend 	fix operator+ (const float &lhs, const fix &rhs);


	private:
	int16_t value;
};

#endif /* FIX_HPP_ */
