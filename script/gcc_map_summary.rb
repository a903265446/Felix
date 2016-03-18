# -*- coding: UTF-8 -*-

#Define the section type
SECTION_NONE = 0
SECTION_ISR_VECTOR = 1
SECTION_FLASH_CONFIG = 2
SECTION_RW_DATA = 3
SECTION_ZI_DATA = 4
SECTION_RO_DATA = 5
SECTION_RO_CODE = 6
SECTION_HEAP = 7
SECTION_STACK = 8

class GccMapSymbol
  @section_type#Int
  @name#String
  @start_address#Int
  @size#Int
  @module_name#String
  def initialize(section_type, name, start_address, size, module_name)
    @section_type = section_type
    @name = name
    @start_address = start_address
    @size = size
    @module_name = module_name
  end

  def get_section_type
    return @section_type
  end

  def get_name
    return @name
  end

  def get_start_address
    return @start_address
  end

  def get_size
    return @size
  end

  def get_module_name
    return @module_name
  end
end

class GccMapSection
  @type#Int
  @start_address
  @size
  @symbol_list#Array, item of which is GccMapSymbol
  def initialize(section_type)
    @type = section_type
    @start_address = 0
    @size = 0
    @symbol_list = Array.new
  end

  def get_symbol_list
    return @symbol_list
  end

  def add_symbol(gcc_map_symbol)
    @symbol_list = (@symbol_list << gcc_map_symbol)
  end

  def get_total_size
    @start_address = @symbol_list[0].get_start_address
    @symbol_list.each { |symbol|
      @size += symbol.get_size
    }
  end

  def get_type
    return @type
  end

  def get_start_address
    return @start_address
  end

  def get_size
    return @size
  end
end

class GccMapModule
  @name#String
  @ro_code_size#Int
  @ro_data_size#Int
  @rw_data_size#Int
  def initialize(module_name)
    @name = module_name
    @ro_code_size = 0
    @ro_data_size = 0
    @rw_data_size = 0
  end

  def get_name
    return @name
  end

  def get_ro_code_size
    return @ro_code_size
  end

  def get_ro_data_size
    return @ro_data_size
  end

  def get_rw_data_size
    return @rw_data_size
  end

  def add_ro_code_size(ro_code_size)
    @ro_code_size += ro_code_size
  end

  def add_ro_data_size(ro_data_size)
    @ro_data_size += ro_data_size
  end

  def add_rw_data_size(rw_data_size)
    @rw_data_size += rw_data_size
  end
end


class GccMap
  @section_array
  @module_hash
  @file_name

  def initialize(file_name)
    @section_array = Array.new
    #Create a array contains 8 section types.
    for section_type in SECTION_ISR_VECTOR .. SECTION_STACK
      gcc_map_section = GccMapSection.new(section_type)
      @section_array = (@section_array << gcc_map_section)
    end
    @module_hash = Hash.new
    @file_name = file_name
  end

  #Get the map file line's section type and distribute the line to corresponding handling function.
  def summarize_symbol
    #Get the line which has symbol information.
    begin
      File.open(@file_name, "r") do |file_handle|
        while line = file_handle.gets do
            #Get gcc map symbol information.
            gcc_map_symbol = get_symbol(file_handle, line)
            if (nil != gcc_map_symbol)
              # puts gcc_map_symbol.get_section_type.to_s(16)
              #Distribute the line content to corresponding section handling function.
              summary_symbol(gcc_map_symbol)
            end
          end
          get_total_size
        end
      rescue Exception => e
        puts "Read file error!"
      end
    end

    #Get section type of the line's symbol
    def get_section_type(line)
      #Get the section type from the line's symbol by checking if the beginning content of the line is ".data",
      #".bss", ".text", ".rodata", ".isr_vector", ".FlashConfig", ".stack", ".heap" and if the line doesn't
      #contain "load address"
      section_type = SECTION_NONE
      if((1 == (line=~/\.data/)) && (!(line=~/load address/)))
        section_type = SECTION_RW_DATA
      end
      if((1 == (line=~/\.bss/)) && (!(line=~/load address/)))
        section_type = SECTION_ZI_DATA
      end
      if(1 == (line=~/\.rodata/))
        section_type = SECTION_RO_DATA
      end
      if(1 == (line=~/\.text/))
        section_type = SECTION_RO_CODE
      end
      if(1 == (line=~/\.isr_vector/))
        section_type = SECTION_ISR_VECTOR
      end
      if(1 == (line=~/\.FlashConfig/))
        section_type = SECTION_FLASH_CONFIG
      end
      if(0 == (line=~/\.stack/))
        section_type = SECTION_STACK
      end
      if(0 == (line=~/\.heap/))
        section_type = SECTION_HEAP
      end

      return section_type
    end

    #Get gcc map symbol information.
    def get_symbol(file_handle, line)
      gcc_map_symbol = !nil

      #Get section type of the line's symbol
      section_type = get_section_type(line)

      if ((SECTION_ISR_VECTOR .. SECTION_STACK) === section_type)
        #Uniformly format the line content
        symbol_attributes = line.split(" ")
        #If line only contain "section type" and "symbol name" field, add next line content.
        line += file_handle.gets if (1 == symbol_attributes.size)
        symbol_attributes = line.split(" ")

        #Get symbol name if line content has symbol name.
        #The sections(.isr_vector, .FlashConfig, .stack, .heap) have no symbol. So set its symbol name as "Total".
        #Still keep the name of the symbols whose name is null in other sections.
        if (nil != symbol_attributes[0].index(".", 1))
          symbol_name_pos = (symbol_attributes[0].index(".", 1) + 1)
          symbol_name = symbol_attributes[0][symbol_name_pos, symbol_attributes[0].length]
        elsif (true == [SECTION_ISR_VECTOR, SECTION_FLASH_CONFIG, SECTION_HEAP, SECTION_STACK].include?(section_type))
          symbol_name = "Total"
        else
          symbol_name = ""
        end
        start_address = symbol_attributes[1].to_i(16)
        size = symbol_attributes[2].to_i(16)
        #Module name is null for stack and heap symbol
        if (3 == symbol_attributes.size)
          module_name = ""
        else
          module_name = symbol_attributes[3]
        end
        #Don't summary the symbols in the /lib/gcc/ directory, size is 0, start address is 0 except for ISR Vector section.
        if ((0 != size) && !(module_name=~/\/lib\//) && ((0 != start_address) && (SECTION_ISR_VECTOR != section_type)) || (SECTION_ISR_VECTOR == section_type))
          gcc_map_symbol = GccMapSymbol.new(section_type, symbol_name, start_address, size, module_name)
        else
          gcc_map_symbol = nil
        end
      else
        gcc_map_symbol = nil
      end

      return gcc_map_symbol
    end

    #Summary each kind of section's symbol information.
    def summary_symbol(gcc_map_symbol)
      #Add the symbol to symbol list array in the section it belongs to.
      section_type = gcc_map_symbol.get_section_type
      # puts section_type.to_s(16)
      #Section object is saved in the (section_type - 1) position in the array.
      @section_array[section_type - 1].add_symbol(gcc_map_symbol)

      #Add the symbol size related information to the module it belongs to.
      module_name = gcc_map_symbol.get_module_name
      #Create key if the key doesn't exist.
      if (false == @module_hash.has_key?(module_name))
        # puts module_name
        # puts "no key"
        gcc_map_module = GccMapModule.new(module_name)
        @module_hash[module_name] = gcc_map_module
      else
        gcc_map_module = @module_hash[module_name]
      end

      size = gcc_map_symbol.get_size
      case section_type
      when SECTION_RO_CODE
        gcc_map_module.add_ro_code_size(size)
      when SECTION_RO_DATA
        gcc_map_module.add_ro_data_size(size)
      when SECTION_RW_DATA, SECTION_ZI_DATA
        gcc_map_module.add_rw_data_size(size)
      else
      end
    end

    def get_total_size
      @section_array.each { |section|
        section.get_total_size
      }
    end

    def print_region_summary
      section_names = Array.new
      section_names = (section_names << ".isr_vector" << ".flash_config" << ".data" << ".bss" << ".rodata" << ".text" << ".heap" << ".stack")
      begin
        #Create a new file based on the original file name to save the summary result.
        result_file_name = (@file_name[0, @file_name.rindex(".")] + "_section.txt")
        puts ("section summary result can be seen from:" + result_file_name)

        #Write the summary information to the file.
        File.open(result_file_name, "w") do |file_handle|
          file_handle.printf("%-20s| %-50s| %20s| %20s| %-50s\r\n", "section type", "symbol name", "start_address", "size", "module_name")
          file_handle.printf("_________________________________________________________________________________________________________________________________________________________________\r\n")
          @section_array.each { |section|
            #Avoid to print two times for .isr_vector, .FlashConfig, .stack, .heap.
            if ((SECTION_RW_DATA .. SECTION_RO_CODE) === section.get_type)
              file_handle.printf("%-20s| %-50s| %12s%08x| %20s| %-50s\r\n", section_names[section.get_type - 1], "Total",
                                 "0x", (section.get_start_address), ("0x" + section.get_size.to_s(16)), "")
            end

            symbol_list = section.get_symbol_list
            symbol_list.each { |symbol|
              file_handle.printf("%-20s| %-50s| %12s%08x| %20s| %-50s\r\n", section_names[symbol.get_section_type - 1], symbol.get_name,
                                 "0x", (symbol.get_start_address), ("0x" + symbol.get_size.to_s(16)), symbol.get_module_name)
            }

            file_handle.printf("%-20s| %-50s| %20s| %20s| %-50s\r\n", "", "", "", "", "")
            file_handle.printf("%-20s| %-50s| %20s| %20s| %-50s\r\n", "", "", "", "", "")
          }
        end
      rescue Exception => e
        puts("Create section summary data file error!")
      end

    end

    def print_module_summary
      begin
        #Create a new file based on the original file name to save the summary result.
        result_file_name = (@file_name[0, @file_name.rindex(".")] + "_module.txt")
        puts ("module summary result can be seen from:" + result_file_name)

        #Write the summary information to the file.
        total_ro_code_size = 0
        total_ro_data_size = 0
        total_rw_data_size = 0
        File.open(result_file_name, "w") do |file_handle|
          file_handle.printf("%-50s| %20s| %20s| %20s\r\n", "module name", "ro code size", "ro data size", "rw data size")
          file_handle.printf("______________________________________________________________________________________________________________________________________________________________\r\n")
          @module_hash.each_value { |gcc_map_module|
            file_handle.printf("%-50s| %20s| %20s| %20s\r\n", gcc_map_module.get_name, gcc_map_module.get_ro_code_size.to_s(10),
                               gcc_map_module.get_ro_data_size.to_s(10), gcc_map_module.get_rw_data_size.to_s(10))

            #Get the total size.
            total_ro_code_size += gcc_map_module.get_ro_code_size
            total_ro_data_size += gcc_map_module.get_ro_data_size
            total_rw_data_size += gcc_map_module.get_rw_data_size
          }
          #Write total size to the file.
          file_handle.printf("______________________________________________________________________________________________________________________________________________________________\r\n")
          file_handle.printf("%-50s| %20s| %20s| %20s\r\n", "Total:", total_ro_code_size, total_ro_data_size, total_rw_data_size)
        end
      rescue Exception => e
        puts("Create module summary data file error!")
      end
    end
  end


  gcc_map = GccMap.new("./adc16_polling_frdmk22f.map")
  # gcc_map = GccMap.new(ARGV[0])
  gcc_map.summarize_symbol
  gcc_map.print_region_summary
  gcc_map.print_module_summary
