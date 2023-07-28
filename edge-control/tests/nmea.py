"""
Parse NMEA sentences from Jackal rosbags, take input from:

    rostopic echo -b *.bag -p /navsat/nmea_sentence_rtk/sentence

    grep "Robot status" record.log | grep nmea | cut -c 66- | jq .gps.gga.nmea -r > nmea
"""

import logging

from edge_control import config
from edge_control.gps.messages import GGA, process

logger = logging.getLogger(__name__)


def main():
    import sys

    site_config = config.SiteConfig.load(sys.argv[1])
    file_name = sys.argv[2]
    for line in open(file_name):
        line = line.strip()
        if line.startswith("%"):
            continue
        # timestamp, nmea = line.split(",", 1)
        timestamp, nmea = 0, line
        try:
            message = process(nmea)
            # print(timestamp, message)
            if isinstance(message, GGA):
                gga = message
                if gga.lat is None or gga.lon is None:
                    logger.debug("GPS position ignored: %r %r", gga.lat, gga.lon)
                    continue

                # Filter on position quality
                if 0 and gga.quality < site_config.gps_quality:
                    logger.warning("GPS position ignored with quality: %r", gga.quality)
                    continue

                x, y = site_config.reference.to_site(gga.lat, gga.lon)
                print("%.3f %.3f %.3f %d" % (gga.time, x, y, gga.quality))
        except:
            print("ERROR", line)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
