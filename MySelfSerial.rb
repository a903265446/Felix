require "serialport"

class MySerial
 attr_accessor :serialName

 def initialize(port)
   port_num = port
   baud_rate = 9600 #class const variable
   data_bits = 8
   stop_bits = 1
   parity = SerialPort::NONE
   @sp = SerialPort.new(port_num, baud_rate, data_bits, stop_bits, parity)
   while true do
     ch = @sp.getc
     putc ch
     @sp.putc ch
   end
   @sp.close
 end
end

puts("Self Serial code ...")
serial1 = MySerial.new(0);
serial1.serialName = "Felix's serial port..."

