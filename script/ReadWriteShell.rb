puts("read/write shell by using ruby. ")
class ReadWriteShellCmd
  def initialize(comments)
    @comments = comments
    @cmdReturn
  end
  def executeShellCmd(cmdStr)
    IO.popen(cmdStr){|f|
       @cmdReturn = f.readlines
       puts @cmdReturn
       size = @cmdReturn.size
       puts("buffer lines number:" + size.to_s)
       for i in 0...size
           puts @cmdReturn[i]
       end
       
       }
  end
end

readWriteShellCmd = ReadWriteShellCmd.new("my comments about the command")
readWriteShellCmd.executeShellCmd("find ./ ")
