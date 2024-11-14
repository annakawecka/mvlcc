#include <mesytec-mvlc.h>
#include <mvlc.h>
#include <mvlc_factory.h>

#include <mvlcc_wrap.h>

using namespace mesytec::mvlc;

struct mvlcc
{
	mesytec::mvlc::CrateConfig config;
	mesytec::mvlc::MVLC mvlc;
	mesytec::mvlc::eth::MVLC_ETH_Interface *ethernet;
	mesytec::mvlc::usb::MVLC_USB_Interface *usb;
};

int readout_eth(eth::MVLC_ETH_Interface *a_eth, uint8_t *a_buffer,
    size_t *bytes_transferred);
int send_empty_request(MVLC *a_mvlc);

static mvlcc_t make_mvlcc(const MVLC &mvlc, const CrateConfig &crateConfig = {})
{
	auto ret = std::make_unique<mvlcc>();
	ret->mvlc = mvlc;
	ret->ethernet = dynamic_cast<eth::MVLC_ETH_Interface *>(ret->mvlc.getImpl());
	ret->usb = dynamic_cast<usb::MVLC_USB_Interface *>(ret->mvlc.getImpl());
	ret->config = crateConfig;
	return ret.release();
}

mvlcc_t
mvlcc_make_mvlc_from_crate_config(const char *configname)
{
	auto m = new mvlcc();
	std::ifstream config(configname);
	if(!config.is_open()) {
		printf("Could not open file '%s'\n", configname);
	}
	m->config = crate_config_from_yaml(config);
	m->mvlc = make_mvlc(m->config);
	m->ethernet = dynamic_cast<eth::MVLC_ETH_Interface *>(
	    m->mvlc.getImpl());
	return m;
}

mvlcc_t
mvlcc_make_mvlc(const char *urlstr)
{
	return make_mvlcc(make_mvlc(urlstr));
}

mvlcc_t mvlcc_make_mvlc_eth(const char *host)
{
	auto m = new mvlcc();
	m->mvlc = make_mvlc_eth(host);
	m->ethernet = dynamic_cast<eth::MVLC_ETH_Interface *>(
	    m->mvlc.getImpl());
	return m;
}

mvlcc_t mvlcc_make_mvlc_usb_from_index(int index)
{
	auto m = new mvlcc();
	m->mvlc = make_mvlc_usb(index);
	m->ethernet = nullptr;
	return m;
}

mvlcc_t mvlcc_make_mvlc_usb_from_serial(const char *serial)
{
	auto m = new mvlcc();
	m->mvlc = make_mvlc_usb(serial);
	m->ethernet = nullptr;
	return m;
}

void
mvlcc_free_mvlc(mvlcc_t a_mvlc)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	m->ethernet = nullptr;
	delete m;
}

int
mvlcc_connect(mvlcc_t a_mvlc)
{
	int rc;
	auto m = static_cast<struct mvlcc *>(a_mvlc);

	/* cancel ongoing readout when connecting */
	m->mvlc.setDisableTriggersOnConnect(true);

	auto ec = m->mvlc.connect();
	rc = ec.value();
	return rc;
}

int
mvlcc_stop(mvlcc_t a_mvlc)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);

	/* perhaps try this a couple of times */
	auto ec = disable_daq_mode_and_triggers(m->mvlc);
	if (ec) {
		printf("'%s'\n", ec.message().c_str());
		return 1;
	}

	return 0;
}

void
mvlcc_disconnect(mvlcc_t a_mvlc)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	m->mvlc.disconnect();
}

int
mvlcc_init_readout(mvlcc_t a_mvlc)
{
	int rc;
	auto m = static_cast<struct mvlcc *>(a_mvlc);

	assert(m->ethernet);

	auto result = init_readout(m->mvlc, m->config, {});

	printf("mvlcc_init_readout\n");
	// std::cout << "init_readout result = " << result.init << std::endl;

	rc = result.ec.value();
	if (rc != 0) {
		printf("init_readout: '%s'\n", result.ec.message().c_str());
		return rc;
	}

	m->ethernet->resetPipeAndChannelStats();

	send_empty_request(&m->mvlc);

	auto ec = setup_readout_triggers(m->mvlc, m->config.triggers);
	if (ec) {
		printf("setup_readout_triggers: '%s'\n", ec.message().c_str());
		return ec.value();
	}

	return rc;
}

int
send_empty_request(MVLC *a_mvlc)
{
	size_t bytesTransferred = 0;

	static const uint32_t empty_request[2] = {
		0xf1000000, 0xf2000000
	};

	auto ec = a_mvlc->getImpl()->write(Pipe::Data,
	    reinterpret_cast<const uint8_t *>(empty_request),
	    2 * sizeof(uint32_t), bytesTransferred);

	printf("send_empty_request: bytesTransferred = %lu\n", bytesTransferred);

	if (ec) {
		printf("Failure writing empty request.\n");
		return ec.value();
	}

	return 0;
}

int
readout_eth(eth::MVLC_ETH_Interface *a_eth, uint8_t *a_buffer,
    size_t a_buffer_len, size_t *a_bytes_transferred)
{
	int rc = 0;
	size_t total_bytes = 0;
	uint8_t *buffer = a_buffer;
	size_t bytes_free = a_buffer_len;
	int cycle = 0;

	printf("  readout_eth: start, bytes_free = %lu\n",
	    bytes_free);
	while (bytes_free >= eth::JumboFrameMaxSize)
	{
		auto result = a_eth->read_packet(Pipe::Data, buffer,
		    bytes_free);
		auto ec = result.ec;
		if (ec == ErrorType::ConnectionError) {
			printf("Connection error.\n");
			return ec.value();
		}
		if (ec == MVLCErrorCode::ShortRead) {
			printf("Short read.\n");
			return ec.value();
		}
		rc = ec.value();
		if (rc != 0) {
			printf("'%s'\n", result.ec.message().c_str());
			return ec.value();
		}
		if (result.leftoverBytes()) {
			printf("Leftover bytes. Bailing out!\n");
			return 1; /* Cannot use ec.value, is 0. */
		}
		buffer += result.bytesTransferred;
		bytes_free -= result.bytesTransferred;
		total_bytes += result.bytesTransferred;

		printf("  readout_eth: cycle = %d, bytes_free = %lu, bytes_tranferred = %d, total_bytes = %lu\n",
		    cycle, bytes_free, result.bytesTransferred, total_bytes);

		++cycle;
	}

	*a_bytes_transferred = total_bytes;
	return rc;
}

int
mvlcc_readout_eth(mvlcc_t a_mvlc, uint8_t **a_buffer, size_t bytes_free)
{
	int rc;
	size_t bytes_transferred;
	uint8_t *buffer;
	auto m = static_cast<struct mvlcc *>(a_mvlc);

	buffer = *a_buffer;

	printf("mvlcc_readout_eth: a_buffer@%p, bytes_free = %lu\n", (void *)*a_buffer, bytes_free);

	rc = readout_eth(m->ethernet, buffer, bytes_free, &bytes_transferred);
	if (rc != 0) {
		printf("Failure in readout_eth %d\n", rc);
		return rc;
	}

	printf("Transferred %lu bytes\n", bytes_transferred);

	*a_buffer += bytes_transferred;

	return rc;
}

mvlcc_addr_width_t
mvlcc_addr_width_from_arg(uint8_t modStr)
{
  mvlcc_addr_width_t mode = mvlcc_A_ERR;
  if (modStr == 16) {
    mode = mvlcc_A16;
  } else if (modStr == 24) {
    mode = mvlcc_A24;
  } else if (modStr == 32) {
    mode = mvlcc_A32;
  } else {
    fprintf(stderr, "Invalid address width: %d\n", modStr);
  }

  return mode;
}

mvlcc_data_width_t
mvlcc_data_width_from_arg(uint8_t modStr)
{
  mvlcc_data_width_t mode = mvlcc_D_ERR;
  if (modStr == 16) {
    mode = mvlcc_D16;
  } else if (modStr == 32) {
    mode = mvlcc_D32;
  } else {
    fprintf(stderr, "Invalid data width: %d\n", modStr);
  }

  return mode;
}

int
mvlcc_single_vme_read(mvlcc_t a_mvlc, uint32_t address, uint32_t * value, uint8_t  amod, uint8_t dataWidth)
{
  int rc;

  auto m = static_cast<struct mvlcc *>(a_mvlc);

  //  mesytec::mvlc::VMEDataWidth m_width = static_cast<mesytec::mvlc::VMEDataWidth>(dataWidth);
  // mesytec::mvlc::u32 * m_value = (mesytec::mvlc::u32 *) value;

  uint8_t mode = mvlcc_addr_width_from_arg(amod);
  uint8_t dWidth = mvlcc_data_width_from_arg(dataWidth);
  mesytec::mvlc::VMEDataWidth m_width = static_cast<mesytec::mvlc::VMEDataWidth>(dWidth);

  auto ec = m->mvlc.vmeRead(address, *value, mode, m_width);
  // auto ec = m->mvlc.vmeRead(address, *m_value, amod, VMEDataWidth::D16);
  rc = ec.value();
  if (rc != 0) {
    printf("Failure in vmeRead %d (%s)\n", rc, ec.message().c_str());
	return rc;
  }

  // printf("\nvalue = %x\n", *value);

  return rc;
}

int
mvlcc_single_vme_write(mvlcc_t a_mvlc, uint32_t address, uint32_t value, uint8_t amod, uint8_t dataWidth)
{
  int rc;

  auto m = static_cast<struct mvlcc *>(a_mvlc);

  uint8_t mode = mvlcc_addr_width_from_arg(amod);
  uint8_t dWidth = mvlcc_data_width_from_arg(dataWidth);
  mesytec::mvlc::VMEDataWidth m_width = static_cast<mesytec::mvlc::VMEDataWidth>(dWidth);

  auto ec = m->mvlc.vmeWrite(address, value, mode, m_width);
  rc = ec.value();
  if (rc != 0) {
    printf("Failure in vmeWrite %d\n", rc);
	return rc;
  }
  //
  return rc;
}

int mvlcc_register_read(mvlcc_t a_mvlc, uint16_t address, uint32_t *value)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	auto ec = m->mvlc.readRegister(address, *value);
	return ec.value();
}

int mvlcc_register_write(mvlcc_t a_mvlc, uint16_t address, uint32_t value)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	auto ec = m->mvlc.writeRegister(address, value);
	return ec.value();
}

static char error_messages[static_cast<size_t>(mesytec::mvlc::MVLCErrorCode::ErrorCodeMax)][256];

const char *mvlcc_strerror(int errnum)
{
	if (errnum < static_cast<int>(mesytec::mvlc::MVLCErrorCode::ErrorCodeMax))
	{
		if (!error_messages[errnum][0])
		{
			auto ec = mesytec::mvlc::make_error_code(static_cast<mesytec::mvlc::MVLCErrorCode>(errnum));
			strncpy(error_messages[errnum], ec.message().c_str(), 255);
		}

		return error_messages[errnum];
	}

	return "<unknown error>";
}

int mvlcc_is_mvlc_valid(mvlcc_t a_mvlc)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	return m->mvlc.isValid();
}

int mvlcc_is_ethernet(mvlcc_t a_mvlc)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	return m->ethernet != nullptr;
}

int mvlcc_is_usb(mvlcc_t a_mvlc)
{
	auto m = static_cast<struct mvlcc *>(a_mvlc);
	return m->usb != nullptr;
}

void mvlcc_set_global_log_level(const char *levelName)
{
	set_global_log_level(spdlog::level::from_str(levelName));
}

void mvlcc_print_mvlc_cmd_counters(FILE *out, mvlcc_t a_mvlc)
{
	assert(out);
	assert(a_mvlc);

	auto m = static_cast<struct mvlcc *>(a_mvlc);
	auto &mvlc = m->mvlc;
	const auto counters = mvlc.getCmdPipeCounters();

	fprintf(out, "super txs: totalTxs=%lu, retries=%lu, cmd txs: totalTxs=%lu, retries=%lu, execRequestsLost=%lu, execResponsesLost=%lu",
		(unsigned long) counters.superTransactionCount,
		(unsigned long) counters.superTransactionRetries,
		(unsigned long) counters.stackTransactionCount,
		(unsigned long) counters.stackTransactionRetries,
		(unsigned long) counters.stackExecRequestsLost,
		(unsigned long) counters.stackExecResponsesLost);

	if (m->ethernet)
	{
		const auto cmdStats = m->ethernet->getPipeStats()[0];
		fprintf(out, ", eth: lostPackets=%lu",
			(unsigned long) cmdStats.lostPackets);
	}
}

void *mvlcc_get_mvlc_object(mvlcc_t a_mvlc)
{
	assert(a_mvlc);

	auto m = static_cast<struct mvlcc *>(a_mvlc);
	return &m->mvlc;
}
