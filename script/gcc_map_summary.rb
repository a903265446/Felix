#!/usr/bin/ruby
# -*- coding: UTF-8 -*-

#Define the section type
SECTION_NONE = 0
SECTION_RW_DATA = 1
SECTION_ZI_DATA = 2
SECTION_RO_DATA = 3
SECTION_RO_CODE = 4
SECTION_ISR_VECTOR = 5
SECTION_FLASH_CONFIG = 6
SECTION_STACK = 7
SECTION_HEAP = 8

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

  def get_section_type()
    return @section_type
  end

  def get_name()
    return @name
  end

  def get_start_address()
    return @start_address
  end

  def get_size()
    return @size
  end

  def get_module_name()
    return @module_name
  end


end

class GccMapSection
  @type#Int
  @symbol_list#Array, item of which is GccMapSymbol
  def initialize(section_type)
    @type = section_type
    @symbol_list = Array.new
  end

  def get_symbol_list()
    return @symbol_list
  end

  def add_symbol(gcc_map_symbol)
    @symbol_list = (@symbol_list << gcc_map_symbol)
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

  def get_name()
    return @name
  end

  def get_ro_code_size()
    return @ro_code_size
  end

  def get_ro_data_size()
    return @ro_data_size
  end

  def get_rw_data_size()
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
    for section_type in SECTION_RW_DATA .. SECTION_HEAP
      gcc_map_section = GccMapSection.new(section_type)
      @section_array = (@section_array << gcc_map_section)
    end
    @module_hash = Hash.new
    @file_name = file_name
  end

  #Get the map file line's section type and distribute the line to corresponding handling function.
  def summarize_symbol()
    #Get the line which has symbol information.
    begin
      File.open(@file_name, "r") do |file_handle|
        while line = file_handle.gets do
            #Get gcc map symbol information.
            gcc_map_symbol = get_symbol(file_handle, line)
            if (nil != gcc_map_symbol)
              # puts gcc_map_symbol.get_section_type().to_s(16)
              #Distribute the line content to corresponding section handling function.
              summary_symbol(gcc_map_symbol)
            end
          end
        end
      rescue Exception => e
        puts "Read file error!"
      end
    end

    #Get section type of the line's symbol
    def get_section_type(line)
      #Get the section type from the line's symbol by checking if the beginning content of the line is ".data",
      #".bss", ".text", ".rodata", ".isr_vector", ".flash_config", ".stack", ".heap" and if the line doesn't
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
      #Get section type of the line's symbol
      section_type = get_section_type(line)

      case section_type
      when SECTION_RW_DATA .. SECTION_FLASH_CONFIG
        #Uniformly format the line content for the section except ".stack" and ".heap"
        #Check if the line has size field.
        symbol_attributes = line.split(" ")
        #If line doesn't contain size field, add next line content.
        if (1 == symbol_attributes.size)
          line += file_handle.gets
        end
        symbol_attributes = line.split(" ")

        #Remove the lines whose size field is 0.
        if (0 != symbol_attributes[2].to_i(16))
          line = symbol_attributes.to_s
          # puts symbol_attributes.to_s
        else
          return nil
        end
      when SECTION_STACK .. SECTION_HEAP
        symbol_attributes = line.split(" ")
        # puts symbol_attributes.to_s
      else
        # puts "unknown section type!"
        return nil
      end

      #Split section name and symbol name from the first array element.
      first_element = symbol_attributes[0]
      if (nil != first_element.index(".", 1))
        symbol_name_pos = (first_element.index(".", 1) + 1)
        symbol_name = first_element[symbol_name_pos, first_element.length]
      else
        symbol_name = ""
      end

      start_address = symbol_attributes[1].to_i(16)
      size = symbol_attributes[2].to_i(16)
      #module name is nil for stack and heap symbol
      if (3 == symbol_attributes.size)
        module_name = "no_name"
      else
        module_name = symbol_attributes[3]
      end

      #Don't summry the symbol in the /lib/gcc/ directory
      if (module_name=~/\/lib\//)
        return nil
        puts "found lib gcc"
      end
      # puts section_type.to_s
      # puts symbol_name
      # puts start_address.to_s(16)
      # puts size.to_s(16)
      # puts module_name
      gcc_map_symbol = GccMapSymbol.new(section_type, symbol_name, start_address, size, module_name)

      return gcc_map_symbol
    end

    #Summary each kind of section's symbol information.
    def summary_symbol(gcc_map_symbol)
      #Add the symbol to symbol list array in the section it belongs to.
      section_type = gcc_map_symbol.get_section_type()
      # puts section_type.to_s(16)
      #Section object is saved in the (section_type - 1) position in the array.
      @section_array[section_type - 1].add_symbol(gcc_map_symbol)

      #Add the symbol size related information to the module it belongs to.
      module_name = gcc_map_symbol.get_module_name()
      if (false == @module_hash.has_key?(module_name))
        puts module_name
        puts "no key"
        gcc_map_module = GccMapModule.new(module_name)
        @module_hash[module_name] = gcc_map_module
      else
        gcc_map_module = @module_hash[module_name]
      end

      size = gcc_map_symbol.get_size()
      case section_type
      when SECTION_RO_CODE
        gcc_map_module.add_ro_code_size(size)
      when SECTION_RO_DATA
        gcc_map_module.add_ro_data_size(size)
      when SECTION_RW_DATA
        gcc_map_module.add_ro_data_size(size)
      else
      end
    end

    def print_region_summary()
      @section_array.each { |section|
        puts "new section"
        symbol_list = section.get_symbol_list()
        symbol_list.each { |symbol|
          puts ("symbol section type:" + symbol.get_section_type.to_s(16) + "          name:" + symbol.get_name() +
            "          start_address:" + symbol.get_start_address().to_s(16) + "           size:" + symbol.get_size().to_s(16) +
            "          module_name:" + symbol.get_module_name())
         }
      }
    end

    def print_module_summary()
      @module_hash.each_value { |gcc_map_module|
        puts ("module name:" + gcc_map_module.get_name() + "           ro code size:" + gcc_map_module.get_ro_code_size.to_s(16) +
          "             ro data size:" + gcc_map_module.get_ro_data_size().to_s(16) + "           rw data size:" + gcc_map_module.get_rw_data_size().to_s(16))
      }
    end
  end


  gcc_map = GccMap.new("./adc16_polling_frdmk22f.map")
  gcc_map.summarize_symbol()
  gcc_map.print_region_summary()
  gcc_map.print_module_summary()
