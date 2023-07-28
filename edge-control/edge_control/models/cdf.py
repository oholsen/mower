from typing import Dict, Optional

from dataclasses import dataclass, field


@dataclass
class File:
    # extractor adds data_set_id
    external_id: str
    timestamp: int
    mime_type: str

    filename: str = ""
    directory: str = ""
    description: str = ""
    metadata: Dict[str, str] = field(default_factory=dict)
    # annotations


@dataclass
class Event:
    # extractor adds data_set_id
    external_id: str
    start_time: int = 0
    end_time: int = 0
    description: str = ""
    type: str = ""
    subtype: str = ""
    metadata: Dict[str, str] = field(default_factory=dict)


@dataclass
class Mission:
    missionId: str
    startTime: int
    name: str  # template name
    status: str
    fault: str = ""
    endTime: int = 0


@dataclass
class GaugeCapture:
    gaugeId: str
    time: int
    missionId: str
    url: str
    mimeType: str
