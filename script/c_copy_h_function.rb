#!/usr/bin/ruby
val1 = "This is variable one"
val2 = "This is variable two"
puts val1
puts val2
val3 = "*"

puts ARGV[0]
puts ARGV[0]

aFile = File.open(ARGV[0], "r");
if (aFile)
    IO.foreach(ARGV[0]){|line| 
        if ((false == (line.include?"* ")) && (false == (line.empty?)))
            puts line
        end
    }
else
    puts "file not exist"
end
