# EDTC-Mini
This is firmware for an STM32 blue pill prototype board. It allows the board to connect to a PC and function a generic joystick with two sliders and up to 24 buttons. The device I wrote this for is a sliding throttle control with 17 buttons.

My blue pill came with a "Chinese clone" STM32F103C8. Remove "upload_flags = -c set CPUTAPID 0x2ba01477" from platformio.ini if you have a chip manufactured by ST.

This project was built with the platform.io plugin for Visual Studio Code using the libopencm3 framework.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
