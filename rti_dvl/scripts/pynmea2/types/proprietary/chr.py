from ... import nmea


class CHR(nmea.ProprietarySentence):
    sentence_types = {}

    def __new__(_cls, manufacturer, data):
        name = manufacturer + data[0]
        cls = _cls.sentence_types.get(name, _cls)
        return super(CHR, cls).__new__(cls)


class CHRS(CHR):
    fields = (
        ('Identity', 'identity', str),
        ('The actual measured angular rate in degrees/s after calibration has been applied', 'rx', float),
        ('The actual measured angular rate in degrees/s after calibration has been applied', 'ry', float),
        ('The actual measured angular rate in degrees/s after calibration has been applied', 'rz', float),
        ('The time at which the last rate gyro data was measured', 'rt', float),
        ('The actual measured acceleration in m/s/s after calibration has been applied', 'ax', float),
        ('The actual measured acceleration in m/s/s after calibration has been applied', 'ay', float),
        ('The actual measured acceleration in m/s/s after calibration has been applied', 'az', float),
        ('The time at which the last acceleration data was measured', 'at', float),
        ('The actual measured magnetic field after calibration has been applied', 'mx', float),
        ('The actual measured magnetic field after calibration has been applied', 'my', float),
        ('The actual measured magnetic field after calibration has been applied', 'mz', float),
        ('The time at which the last magnetometer data was measured', 'mt', float)
    )


class CHRA(CHR):
    fields = (
        ('Identity', 'identity', str),
        ('The estimated roll angle', 'r', float),
        ('The estimated pitch angle', 'p', float),
        ('The estimated yaw angle', 'y', float),
        ('The time that the Euler Angles were measured', 't', float)
    )
