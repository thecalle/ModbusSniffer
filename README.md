ModbusSniffer
=============

Modbus RTU packet sniffer

Print all packets on bus from either slave or master.
Useful for sniffing packets between two devices to ensure correct operation.

Simple command line tool that prints results to the terminal.

Documentation
-------------

```text
Usage:  
  python modbus_sniffer.py [arguments]

Arguments:  
  -p, --port        select the serial port (Required)  
  -b, --baudrate    set the communication baud rate, default = 9600 (Option)  
  -t, --timeout     overrite the calculated inter frame timeout, default = 0.0034375s (Option)
  -l, --logfile     save terminal output to file
  -h, --help        print the documentation
```

Project Informations
--------------------

### - Languages

[![GitHub language count](https://img.shields.io/github/languages/count/ekristoffe/ModbusSniffer)](README.md)  
[![GitHub top language](https://img.shields.io/github/languages/top/ekristoffe/ModbusSniffer)](README.md)

### - License

[![GitHub](https://img.shields.io/github/license/ekristoffe/ModbusSniffer)](https://github.com/ekristoffe/ModbusSniffer/blob/main/LICENSE)

### - Release Note

* 1.0.0 First release

Disclaimer
----------

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
