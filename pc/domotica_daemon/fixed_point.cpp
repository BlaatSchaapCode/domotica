#include <stdint.h>
#include <stdio.h>

// Testing if we can emulate fixed point values in C++
// such we can use this on the linux end, when the microcontroller uses real
// fixed point math.

// https://www.educative.io/blog/overloading-operator-in-cpp



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


fix operator+ (const float &lhs, const fix &rhs) {
	fix result;
	result.value =  (lhs * 256.0f)  + rhs.value;
	return result;
}

//int main(void) {
//	fix fi;
//	float fl;
//
//	fi = 6.5;
//	fl = fi;
//	fi = fl;
//	fi = fl + fi;
//	fi = fi + fl;
//
//
//
//	printf("Test %f\n", fl);
//	printf("Sizeof fix is %d\n", sizeof(fix) );
//
//
//}
