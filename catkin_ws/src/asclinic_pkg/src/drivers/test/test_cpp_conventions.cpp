#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <cmath>


int main()
{
	printf("\nStatic int cast\n");

	float a_float = 0.7;
	int a_int_cast = static_cast<int>(a_float);
	int a_int_cast_round = static_cast<int>(std::round(a_float));

	float b_float = -0.2;
        int b_int_cast = static_cast<int>(b_float);
	int b_int_cast_round = static_cast<int>(std::round(b_float));

	float c_float = -0.7;
        int c_int_cast = static_cast<int>(c_float);
	int c_int_cast_round = static_cast<int>(std::round(c_float));

        printf("a_float = %f, a_int_cast = %d, a_int_cast_round = %d\n", a_float, a_int_cast, a_int_cast_round);

	printf("b_float = %f, b_int_cast = %d, b_int_cast_round = %d\n", b_float, b_int_cast, b_int_cast_round);

	printf("c_float = %f, c_int_cast = %d, c_int_cast_round = %d\n", c_float, c_int_cast, c_int_cast_round);

	// Return
	return 0;
}
