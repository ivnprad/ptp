#include "PtpClient.h"
#include "PtpServer.h"

#include <span> 
#include <filesystem> 
#include <boost/program_options.hpp>
#include <iostream> 

namespace
{
	struct ProgramOptions
	{
		std::filesystem::path ProgramName;
		std::string IpAddress;
		bool Client{ false };
	};

    boost::program_options::variables_map GetProgramArguments(
        std::span<const char* const> args,
        const boost::program_options::options_description& description)
    {
        namespace po = boost::program_options;
    
        po::variables_map vm;
    
        try 
        {
            po::store(po::command_line_parser(static_cast<int>(args.size()), args.data())
                          .options(description)
                          .run(),
                      vm);
            po::notify(vm);
        } 
        catch (const po::error& ex) 
        {
            throw std::runtime_error(std::string("Argument parsing error: ") + ex.what());
        }
    
        return vm;
    }

	ProgramOptions ReadProgramOptions(std::span<const char* const> args)
	{
		constexpr auto c_clientArgument{ "Client" };
		constexpr auto c_ipArgument{ "IpAddress" };

		boost::program_options::options_description description("Client Server");
		description.add_options()
			(c_ipArgument, boost::program_options::value<std::string>(),
			"provide ip address of the the server")
			(c_clientArgument, boost::program_options::bool_switch()->default_value(false),
			"server as default, write --Client if you want to change");

		const auto arguments{ GetProgramArguments(args, description) };
		ProgramOptions programOptions;
		programOptions.ProgramName = std::filesystem::path(std::size(args) > 0 ? args[0] : "");
		programOptions.Client = arguments[c_clientArgument].as<bool>();

        if (programOptions.Client && !arguments.count(c_ipArgument))
            throw std::runtime_error("--IpAddress is required when --Client is specified.");

		if (!arguments.count(c_ipArgument))
			return programOptions;

		programOptions.IpAddress = arguments[c_ipArgument].as<std::string>();
		return programOptions;
	}
}

// Main function
#pragma warning(suppress: 26461) // The pointer argument 'argv' for function 'main' can be marked as a pointer to const (con.3).
#pragma warning(suppress: 26485) // Expression 'argv': No array to pointer decay (bounds.3).
int main(int argc, char* argv[])
{
	try
	{
		boost::asio::io_context ioContext;
		const auto programOptions{ ReadProgramOptions(std::span(argv, argc)) };
		if (programOptions.Client)
		{
			PTP::Client client(ioContext, PTP::c_serverIP, PTP::c_clientIP);
			ioContext.run();
		}
		else
		{
			PTP::Server server(ioContext, PTP::c_serverIP, PTP::c_ptpEventPort, PTP::c_ptpGeneralPort);
			ioContext.run();
		}

		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

}