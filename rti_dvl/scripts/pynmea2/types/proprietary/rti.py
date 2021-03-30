from ... import nmea


class RTI(nmea.ProprietarySentence):
    sentence_types = {}

    def __new__(_cls, manufacturer, data):
        name = manufacturer + data[0]
        cls = _cls.sentence_types.get(name, _cls)
        return super(RTI, cls).__new__(cls)


class RTI01(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Start time of this sample in hundreds of seconds since power up or user reset', 'time', int),
        ('Sample number', 'number', int),
        ('Temperature in hundreds of degrees Celsius', 'temperature', int),
        ('Bottom track X velocity component mm/s', 'x', int),
        ('Bottom track Y velocity component mm/s', 'y', int),
        ('Bottom track Z velocity component mm/s', 'z', int),
        ('Depth below transducer in mm', 'depth', int),
        ('Water mass X velocity component mm/s', '_blank'),
        ('Water mass Y velocity component mm/s', '_blank'),
        ('Water mass Z velocity component mm/s', '_blank'),
        ('Depth of water mass measurement in mm', '_blank'),
        ('Built in test and status bits in hexadecimal0000', '_blank'),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI02(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Start time of this sample in hundreds of seconds since power up or user reset', 'time', int),
        ('Sample number', 'number', int),
        ('Temperature in hundreds of degrees Celsius', 'temperature', int),
        ('Bottom track East velocity component mm/s', 'east', int),
        ('Bottom track North velocity component mm/s', 'north', int),
        ('Bottom track Up velocity component mm/s', 'up', int),
        ('Depth below transducer in mm', 'depth', int),
        ('Water mass East velocity component mm/s', '_blank'),
        ('Water mass North velocity component mm/s', '_blank'),
        ('Water mass Up velocity component mm/s', '_blank'),
        ('Depth of water mass measurement in mm', '_blank'),
        ('Built in test and status bits in hexadecimal0000', '_blank'),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI03(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Start time of this sample in hundreds of seconds since power up or user reset', 'time', int),
        ('Sample number', 'number', int),
        ('Temperature in hundreds of degrees Celsius', 'temperature', int),
        ('Bottom track X velocity component mm/s', 'x', int),
        ('Bottom track Y velocity component mm/s', 'y', int),
        ('Bottom track Z velocity component mm/s', 'z', int),
        ('Bottom track Q velocity component mm/s', 'q', int),
        ('Depth below transducer in mm', 'depth', int),
        ('Water mass X velocity component mm/s', '_blank'),
        ('Water mass Y velocity component mm/s', '_blank'),
        ('Water mass Z velocity component mm/s', '_blank'),
        ('Water mass Q velocity component mm/s', '_blank'),
        ('Depth of water mass measurement in mm', '_blank'),
        ('Built in test and status bits in hexadecimal0000', '_blank'),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI30(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Heading used during the bottom track ping', 'heading', float),
        ('Pitch used during the bottom track ping', 'pitch', float),
        ('Roll used during the bottom track ping', 'roll', float),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI31(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Heading used during the water mass ping', 'heading', float),
        ('Pitch used during the water mass ping', 'pitch', float),
        ('Roll used during the water mass ping', 'roll', float),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI32(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Heading used during the bottom track ping', 'heading', float),
        ('Pitch used during the bottom track ping', 'pitch', float),
        ('Roll used during the bottom track ping', 'roll', float),
        ('Pressure in BAR', 'pressure', float),
        ('Water Temperature in degrees C', 'temperature', float),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI33(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Heading used during the water mass ping', 'heading', float),
        ('Pitch used during the water mass ping', 'pitch', float),
        ('Roll used during the water mass ping', 'roll', float),
        ('Pressure in BAR', 'pressure', float),
        ('Water Temperature in degrees C', 'temperature', float),
        ('Sub System', '_blank'),
        ('Sub System index', '_blank')
    )


class RTI34(RTI):
    fields = (
        ('Identity', 'identity', str),
        ('Heading', 'heading', float),
        ('Pitch', 'pitch', float),
        ('Roll', 'roll', float)
    )
