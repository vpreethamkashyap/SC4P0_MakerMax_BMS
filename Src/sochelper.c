/**
 ******************************************************************************
 * File Name          : SOCHELPER.C
 * Description        : SOC HELPER FILES
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 MAKERMAX INC.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of MAKERMAX INC. nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

float lookupSOCByOCV(float restedOcvInput, float* ocvTable, int tableSize, float* socTable)
{
	/* number of elements in the array */
	int count = tableSize;

	int i;
	float dx, dy;

	if (restedOcvInput < ocvTable[0]) {
		/* x is less than the minimum element
		 * handle error here if you want */
		return socTable[0]; /* return minimum element */
	}

	if (restedOcvInput > ocvTable[count - 1]) {
		return socTable[count - 1]; /* return maximum */
	}

	/* find i, such that ocvTable[i] <= x < ocvTable[i+1] */
	for (i = 0; i < count - 1; i++) {
		if (ocvTable[i + 1] > restedOcvInput) {
			break;
		}
	}

	/* interpolate */
	dx = ocvTable[i + 1] - ocvTable[i];
	dy = socTable[i + 1] - socTable[i];
	float socByOcv = socTable[i] + (restedOcvInput - ocvTable[i]) * dy / dx;
	return socByOcv;
}




