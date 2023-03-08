/* Copyright (C) 2015 - 2021 National Aeronautics and Space Administration. All Foreign Rights are Reserved to the U.S. Government.

   This software is provided "as is" without any warranty of any, kind either express, implied, or statutory, including, but not
   limited to, any warranty that the software will conform to, specifications any implied warranties of merchantability, fitness
   for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the program, or
   any warranty that the software will be error free.

   In no event shall NASA be liable for any damages, including, but not limited to direct, indirect, special or consequential damages,
   arising out of, resulting from, or in any way connected with the software or its documentation.  Whether or not based upon warranty,
   contract, tort or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software,
   documentation or services provided hereunder

   ITC Team
   NASA IV&V
   ivv-itc@lists.nasa.gov
*/

#include <simulator_terminal.hpp>

#include <iostream>
#include <thread>
#include <memory>
#include <stdexcept>

#include <readline/readline.h>
#include <readline/history.h>

#include <ItcLogger/Logger.hpp>
#include <Client/Bus.hpp>
#include <Client/DataNode.hpp>
#include <Utility/Buffer.hpp>
#include <Utility/BufferOverlay.hpp>
#include <I2C/Client/I2CMaster.hpp>
#include <Can/Client/CanMaster.hpp>
#include <Spi/Client/SpiMaster.hpp>
#include <Uart/Client/Uart.hpp>

#include <sim_hardware_model_factory.hpp>
#include <sim_i_hardware_model.hpp>
#include <sim_config.hpp>

#include <boost/algorithm/string/find.hpp>

#include <bus_connections.hpp>

namespace Nos3
{
    REGISTER_HARDWARE_MODEL(SimTerminal,"SimTerminal");

    ItcLogger::Logger *sim_logger;


    // Constructors
    SimTerminal::SimTerminal(const boost::property_tree::ptree& config) : SimIHardwareModel(config),
        _other_node_name(config.get("simulator.hardware-model.other-node-name", "time")),
        _bus_name(config.get("simulator.hardware-model.bus.name", "command")),
        _current_in_mode((config.get("simulator.hardware-model.input-mode", "").compare("HEX") == 0) ? HEX : ASCII),
        _current_out_mode((config.get("simulator.hardware-model.output-mode", "").compare("HEX") == 0) ? HEX : ASCII),
        _long_prompt(true)
    {
        std::string bus_type = config.get("simulator.hardware-model.bus.name", "command");
        if (!set_bus_type(bus_type)) {
            sim_logger->error("Invalid bus type setting %s.  Setting bus type to COMMAND.", bus_type.c_str());
        }
        _nos_connection_string = config.get("common.nos-connection-string", "tcp://127.0.0.1:12001");
        _command_node_name = config.get("simulator.hardware-model.terminal-node-name", "terminal");

        _connection_strings["default"] = _nos_connection_string;

        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, config.get_child("simulator.hardware-model.other-nos-connections")) 
        {
            std::string name = v.second.get("name", "");
            std::string connection_string = v.second.get("connection-string", "");
            if ((name.compare("") != 0) && (name.compare("default") != 0)) {
                _connection_strings[name] = connection_string;
            }
        }

        _active_connection_name = "default";

        reset_bus_connection();

        if (config.get_child_optional("simulator.hardware-model.startup-commands")) 
        {
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, config.get_child("simulator.hardware-model.startup-commands")) 
            {
                if (v.first.compare("command") == 0) {
                    process_command(v.second.data());
                }
            }
        }
    }

    /// @name Mutating public worker methods
    //@{
    /// \brief Runs the server, creating the NOS Engine bus and the transports for the simulator and simulator client to connect to.
    void SimTerminal::run(void)
    {
        try
        {
            handle_input(); // when handle_input returns... it is time to quit
        }
        catch(...)
        {
            Nos3::sim_logger->error("SimTerminal::run:  Exception caught!");
        }
    }
    //@}

    void SimTerminal::write_message_to_cout(const char* buf, size_t len){
        for (unsigned int i = 0; i < len; i++) {
            if (_current_out_mode == HEX) {
                std::cout << " 0x" << convert_hexhexchar_to_asciihexchars(buf[i]);
            } else {
                std::cout << buf[i];
            }
        }

        std::cout << std::endl;
    }

    void SimTerminal::write_message_to_cout(const NosEngine::Common::Message& msg)
    {
        NosEngine::Common::DataBufferOverlay dbf(const_cast<NosEngine::Utility::Buffer&>(msg.buffer));
        write_message_to_cout(dbf.data, dbf.len);
    }

    void SimTerminal::reset_bus_connection(){
         // Figure out where _old_connection lives and what data structure it belongs to
         if (_bus_type == I2C){
            int master_address;
            try{
                master_address = stoi(_command_node_name);
            }catch(std::invalid_argument e){
                master_address = 127;
                _command_node_name = "127";
                std::cout << "\"" << _command_node_name << "\" is not a valid I2C address for the terminal. Defaulting to 127." << std::endl;
            }
            // if old connection exists, free it
            if (_bus_connection.get() != nullptr) {
                BusConnection* old = _bus_connection.release();
                delete old;
                // Nos3::sim_logger->debug("reset_bus_connection: deleting old bus connection");
            }
            
            I2CConnection* i2c = new I2CConnection(master_address, _nos_connection_string, _bus_name);
            _bus_connection.reset(i2c);
        } else if (_bus_type == CAN){
            int master_identifier;
            try{
                master_identifier = stoi(_command_node_name);
            }catch(std::invalid_argument e){
                master_identifier = 127;
                _command_node_name = "127";
                std::cout << "\"" << _command_node_name << "\" is not a valid CAN identifier for the terminal. Defaulting to 127." << std::endl;
            }
            // if old connection exists, free it
            if (_bus_connection.get() != nullptr) {
                BusConnection* old = _bus_connection.release();
                delete old;
                // Nos3::sim_logger->debug("reset_bus_connection: deleting old bus connection");
            }
            
            CANConnection* can = new CANConnection(master_identifier, _nos_connection_string, _bus_name);
            _bus_connection.reset(can);
        } else if(_bus_type == SPI){
            // if old connection exists, free it
            if (_bus_connection.get() != nullptr) {
                BusConnection* old = _bus_connection.release();
                delete old;
                // Nos3::sim_logger->debug("reset_bus_connection: deleting old bus connection");
            }
           _bus_connection.reset(new SPIConnection(_nos_connection_string, _bus_name));
        }else if(_bus_type == UART){
            // if old connection exists, free it
            if (_bus_connection.get() != nullptr) {
                BusConnection* old = _bus_connection.release();
                delete old;
                // Nos3::sim_logger->debug("reset_bus_connection: deleting old bus connection");
            }
            _bus_connection.reset(new UartConnection(this, _command_node_name, _nos_connection_string, _bus_name));
        }else{ // not differentiating between BASE and COMMAND types... yet
            // if old connection exists, free it
            if (_bus_connection.get() != nullptr) {
                BusConnection* old = _bus_connection.release();
                delete old;
                // Nos3::sim_logger->debug("reset_bus_connection: deleting old bus connection");
            }
            _bus_connection.reset(new BaseConnection(this, _command_node_name, _nos_connection_string, _bus_name));
        }
        _bus_connection->set_target(_other_node_name);
    }

    bool SimTerminal::process_command(std::string input){
        std::string in_upper = input;
        boost::to_upper(in_upper);

        if (in_upper.compare(0, 4, "HELP") == 0) 
        {
            std::cout << "This is help for the simulator terminal program." << std::endl;
            std::cout << "  The prompt shows the <simulator terminal node name@simulator bus name> and <simulator node being commanded> " << std::endl;
            std::cout << "  Commands:" << std::endl;
            std::cout << "    HELP - Displays this help" << std::endl;
            std::cout << "    QUIT - Exits the program" << std::endl;
            std::cout << "    SET SIMNODE <sim node> - Sets the simulator node being commanded to '<sim node>'" << std::endl;
            std::cout << "    SET SIMBUS <sim bus> - Sets the simulator bus for the simulator node being commanded to '<sim bus>'" << std::endl;
            std::cout << "    SET SIMBUSTYPE <bus type> - Sets the simulator bus type for the simulator node being commanded to '<bus type>'" << std::endl; 
            std::cout << "        (BASE, I2C, CAN, SPI, UART, COMMAND are valid)" << std::endl;
            std::cout << "    SET TERMNODE <term node> - Sets the name of this terminal's node to '<term node>'" << std::endl;
            std::cout << "    SET <ASCII|HEX> <IN|OUT> - Sets the terminal mode to ASCII mode or HEX mode; optionally IN or OUT only" << std::endl;
            std::cout << "    SET PROMPT <LONG|SHORT> - Sets the prompt to long or short format" << std::endl;
            std::cout << "    LIST NOS CONNECTIONS - Lists all of the known NOS Engine connection strings along with a name for selecting them" << std::endl;
            std::cout << "    SET NOS CONNECTION <name> - Sets the NOS Engine connection to the one associated with <name> (initially \"default\")" << std::endl;
            std::cout << "    ADD NOS CONNECTION <name> <uri> - Adds NOS Engine URI connection string <uri> to the list of known connection strings and associates it with <name>" << std::endl;
            std::cout << "    WRITE <data> - Writes <data> to the current node. Interprets <data> as ascii or hex depending on input setting." << std::endl;
            std::cout << "    READ <length> - Reads the given number of bytes from the current node. Only works on SPI and I2C buses." << std::endl;
            std::cout << "    TRANSACT <read length> <data> - Performs a transaction. Sends the given data, and expects a return value of the given length." << std::endl;
            std::cout << "             Interprets everything after the first space after <read length> as data to be written." << std::endl;
        } 
        else if (in_upper.compare(0, 12, "SET SIMNODE ") == 0) 
        {
            _other_node_name = input.substr(12, input.size() - 12);
            _bus_connection->set_target(_other_node_name);
        } 
        else if (in_upper.compare(0, 11, "SET SIMBUS ") == 0) 
        {
            std::string new_command_bus_name = input.substr(11, input.size() - 11);
            if(new_command_bus_name.compare(_bus_name) != 0){
                _bus_name = new_command_bus_name;
                reset_bus_connection();
            }else{
                std::cout << "Already on bus " << _bus_name << std::endl;
            }
        } 
        else if (in_upper.compare(0, 15, "SET SIMBUSTYPE ") == 0) 
        {
            std::string new_command_bus_type = input.substr(15, input.size() - 15);
            if (set_bus_type(new_command_bus_type)) {
                reset_bus_connection();
            } else {
                 std::cout << "Invalid bus type setting " << new_command_bus_type << ".  Not changing bus type." << std::endl;
            }
        } 
        else if (in_upper.compare(0, 13, "SET TERMNODE ") == 0) 
        {
            _command_node_name = input.substr(13, input.length());
            reset_bus_connection();
        } 
        else if (in_upper.compare(0, 9, "SET ASCII") == 0) 
        {
            if ((input.size() == 9) || (in_upper.compare(0, 12, "SET ASCII IN") == 0)) _current_in_mode = ASCII;
            if ((input.size() == 9) || (in_upper.compare(0, 13, "SET ASCII OUT") == 0)) _current_out_mode = ASCII;
        } 
        else if (in_upper.compare(0, 7, "SET HEX") == 0) 
        {
            if ((input.size() == 7) || (in_upper.compare(0, 10, "SET HEX IN") == 0)) _current_in_mode = HEX;
            if ((input.size() == 7) || (in_upper.compare(0, 11, "SET HEX OUT") == 0)) _current_out_mode = HEX;
        } 
        else if (in_upper.compare(0, 11, "SET PROMPT ") == 0) 
        {
            std::string prompt_type = input.substr(11, input.size() - 11);
            boost::to_upper(prompt_type);
            if (prompt_type.compare("LONG") == 0) _long_prompt = true;
            else if (prompt_type.compare("SHORT") == 0) _long_prompt = false;
            else std::cout << "Invalid prompt length specified" << prompt_type << std::endl;
        }
        else if (in_upper.compare(0, 20, "LIST NOS CONNECTIONS") == 0) 
        {
            for (std::map<std::string, std::string>::const_iterator it = _connection_strings.begin(); it != _connection_strings.end(); it++)
                std::cout << "    name=" << it->first << ", connection string=" << it->second << std::endl;
        }
        else if (in_upper.compare(0, 19, "SET NOS CONNECTION ") == 0) 
        {
            std::string name = input.substr(19, input.size() - 19);
            try {
                std::string connection_string = _connection_strings.at(name);
                if (connection_string.compare(_nos_connection_string) != 0) {
                    _nos_connection_string = connection_string;
                    _active_connection_name = name;
                    reset_bus_connection();
                }
            } catch (std::exception&) {
                std::cout << "Invalid connection name \"" << name << "\"" << std::endl;
            }
        }
        else if (in_upper.compare(0, 19, "ADD NOS CONNECTION ") == 0) 
        {
            std::string token;
            std::vector<std::string> tokens;
            std::stringstream ss(input);
            while (std::getline(ss, token, ' ')) {
                tokens.push_back(token);
            }
            if (tokens.size() == 5) {
                _connection_strings[tokens[3]] = tokens[4];
            } else {
                std::cout << "Invalid ADD NOS CONNECTION command \"" << input << "\".  Type \"HELP\" for help." << std::endl;
            }
        }
        else if (in_upper.compare(0, 4, "QUIT") == 0) 
        {
            return true;
        }
        else if (in_upper.compare(0, 6, "WRITE ") == 0)
        {
            if(_bus_connection.get() == nullptr){
                std::cout << "Connection has not been instantiated. Connect to a bus with SET SIMBUS." << std::endl;
                return false;
            }
            std::string buf = input.substr(6, input.length() - 6);
            if(_current_in_mode == HEX){
                buf = convert_asciihex_to_hexhex(buf);
            }
            int wlen = buf.length();

            try{
                _bus_connection->write(buf.c_str(), wlen);
            }catch (std::runtime_error e){
                std::cout << e.what() << std::endl;
            }
            
        }
        else if (in_upper.compare(0, 5, "READ ") == 0)
        {
            if(_bus_connection.get() == nullptr){
                std::cout << "Connection has not been instantiated. Connect to a bus with SET SIMBUS." << std::endl;
                return false;
            }
            char buf[255];
            int len;
            std::string len_string = input.substr(5, input.length());
            try{
                len = stoi(len_string);
            }catch (std::invalid_argument e){
                len = 0;
            }

            try {
                _bus_connection->read(buf, len);
                write_message_to_cout(buf, len);
            }catch (std::runtime_error e){
                std::cout << e.what() << std::endl;
            }
            
        }
        else if (in_upper.compare(0, 9, "TRANSACT ") == 0)
        {
            if(_bus_connection.get() == nullptr){
                std::cout << "Connection has not been instantiated. Connect to a bus with SET SIMBUS." << std::endl;
                return false;
            }
            char rbuf[255];
            int rlen;
            int numberStart = 9;
            int dataStart;

            boost::iterator_range<std::string::iterator> r = boost::find_nth(input, " ", 1);
            dataStart = std::distance(input.begin(), r.begin()) + 1;

            try {
                rlen = stoi(input.substr(numberStart, dataStart - numberStart));
            }catch (std::invalid_argument){
                std::cout << "\"" << input.substr(numberStart, dataStart - numberStart) << "\" is not a valid number." << std::endl;
                return false;
            }

            //std::cout << "rlen: " << rlen << ", Data: " << input.substr(dataStart, input.length() - dataStart) << std::endl;

            std::string wbuf = input.substr(dataStart, input.length() - dataStart);
            if(_current_in_mode == HEX){
                wbuf = convert_asciihex_to_hexhex(wbuf);
            }
            int wlen = wbuf.length();
            try {
                _bus_connection->transact(wbuf.c_str(), wlen, rbuf, rlen);
                write_message_to_cout(rbuf, rlen);
            }catch (std::runtime_error e){
                std::cout << e.what() << std::endl;
            }
        }
        else if (input.length() > 0)
        {
            std::cout << "Unrecognized command \"" << input << "\". Type \"HELP\" for help." << std::endl;
        }
        return false;
    }

    bool SimTerminal::getline(const std::string& prompt, std::string& input)
    {
        bool retval = true;
        input.clear();
        char *line_read = readline(prompt.c_str());
        if (line_read) {
            if (*line_read) add_history(line_read); // don't add blank lines
            input.append(line_read);
            free(line_read);
            retval = true;
        } else {
            retval = false;
        }
        return retval;
    }

    void SimTerminal::handle_input(void)
    {
        std::string input, in_upper;
        std::cout << "This is the simulator terminal program.  Type 'HELP' for help." << std::endl << std::endl;
        while(getline(string_prompt(), input)) // keep looping and getting the next command line
        {
            bool result = process_command(input);
            if(result){
                break;
            }
        }

        std::cout << "SimTerminal is quitting!" << std::endl;
    }

    std::string SimTerminal::string_prompt(void)
    {
        std::stringstream ss;
        if (_long_prompt) {
            ss  << _command_node_name 
                << "-" << _active_connection_name
                << "<" << _other_node_name << ">" 
                << ":(" << _bus_type_string[_bus_type] << ")" << _bus_name
                << ":[" << mode_as_string() << "] $ ";
        } else {
            ss  <<          _command_node_name
                << "-->" << _other_node_name
                << "@(" << _bus_type_string[_bus_type] << ")" << _bus_name
                << "[" << mode_as_string() << "] $ ";
        }
        return ss.str();
    }

    std::string SimTerminal::mode_as_string(void)
    {
        std::string mode;
        if (_long_prompt) {
            if (_current_in_mode == ASCII) mode.append("IN=ASCII:");
            if (_current_in_mode == HEX) mode.append("IN=HEX:");
            if (_current_out_mode == ASCII) mode.append("OUT=ASCII");
            if (_current_out_mode == HEX) mode.append("OUT=HEX");
        } else {
            if (_current_in_mode == ASCII) mode.append("I=A:");
            if (_current_in_mode == HEX) mode.append("I=H:");
            if (_current_out_mode == ASCII) mode.append("O=A");
            if (_current_out_mode == HEX) mode.append("O=H");
        }
        return mode;
    }

    std::string SimTerminal::convert_hexhexchar_to_asciihexchars(uint8_t in)
    {
        std::string out;
        uint8_t inupper = (in & 0xF0) >> 4;
        uint8_t inlower = in & 0x0F;
        out.push_back(convert_hexhexnibble_to_asciihexchar(inupper));
        out.push_back(convert_hexhexnibble_to_asciihexchar(inlower));
        return out;
    }

    char SimTerminal::convert_hexhexnibble_to_asciihexchar(uint8_t in)
    {
        char out = '.';
        if ((0x0 <= in) && (in <= 0x9)) out = in - 0x0 + '0';
        if ((0xA <= in) && (in <= 0xF)) out = in - 0xA + 'A';
        return out;
    }

    std::string SimTerminal::convert_asciihex_to_hexhex(std::string in)
    {
        std::string out;
        in.push_back('0'); // in case there are an odd number of characters, tack a 0 on the end
        for (size_t i = 0; i < in.size() - 1; i += 2) {
            out.push_back(convert_asciihexcharpair_to_hexhexchar(in[i], in[i+1]));
        }
        return out;
    }

    uint8_t SimTerminal::convert_asciihexcharpair_to_hexhexchar(char in1, char in2)
    {
        uint8_t outupper = convert_asciihexchar_to_hexhexchar(in1);
        uint8_t outlower = convert_asciihexchar_to_hexhexchar(in2);
        uint8_t out = ((outupper << 4) + outlower);
        return out;
    }

    uint8_t SimTerminal::convert_asciihexchar_to_hexhexchar(char in)
    {
        uint8_t out = 0;
        if (('0' <= in) && (in <= '9')) out = in - '0';
        if (('A' <= in) && (in <= 'F')) out = in - 'A' + 10;
        if (('a' <= in) && (in <= 'f')) out = in - 'a' + 10;
        return out;
    }

    bool SimTerminal::set_bus_type(std::string type)
    {
        bool succeeded = true;
        boost::to_upper(type);
        if (type.compare("BASE") == 0) {
            _bus_type = BASE;
        } else if (type.compare("I2C") == 0) {
            _bus_type = I2C;
        } else if (type.compare("CAN") == 0) {
            _bus_type = CAN;
        } else if (type.compare("SPI") == 0) {
            _bus_type = SPI;
        } else if (type.compare("UART") == 0) {
            _bus_type = UART;
        } else if (type.compare("COMMAND") == 0) {
            _bus_type = COMMAND;
        } else {
            succeeded = false;
        }
        return succeeded;
    }
}
