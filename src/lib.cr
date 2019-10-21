@[Link(ldflags: "-lgps")]
lib LibGps
  GPSD_API_MAJOR_VERSION =                                      8
  GPSD_API_MINOR_VERSION =                                      0
  GPS_AMBIGUITY_MODULUS  =                             299792.458
  GPS_LN2                = 0.693147180559945309417232121458176568
  GPS_PATH_MAX           =                                    128
  GPS_PI                 =   3.1415926535897932384626433832795029
  alias GpsMaskT = Uint64T
  alias SocketT = LibC::Int
  alias TimeT = X__TimeT
  alias TimestampT = LibC::Double
  alias Uint32T = X__Uint32T
  alias Uint64T = X__Uint64T
  alias Uint8T = X__Uint8T
  alias X_IoCodecvt = Void
  alias X_IoLockT = Void
  alias X_IoMarker = Void
  alias X_IoWideData = Void
  alias X__Off64T = LibC::Long
  alias X__OffT = LibC::Long
  alias X__SyscallSlongT = LibC::Long
  alias X__TimeT = LibC::Long
  alias X__Uint32T = LibC::UInt
  alias X__Uint64T = LibC::ULong
  alias X__Uint8T = UInt8
  fun gps_clear_att(x0 : AttitudeT*)
  fun gps_clear_dop(x0 : DopT*)
  fun gps_clear_fix(x0 : GpsFixT*)
  fun gps_close(x0 : GpsDataT*) : LibC::Int
  fun gps_data(x0 : GpsDataT*) : LibC::Char*
    fun gps_enable_debug(x0 : LibC::Int, x1 : File*)
  fun gps_errstr(x0 : LibC::Int) : LibC::Char*
    fun gps_mainloop(x0 : GpsDataT*, x1 : LibC::Int, x2 : (GpsDataT* -> Void)) : LibC::Int
  fun gps_maskdump(x0 : GpsMaskT) : LibC::Char*
    fun gps_merge_fix(x0 : GpsFixT*, x1 : GpsMaskT, x2 : GpsFixT*)
  fun gps_open(x0 : LibC::Char*, x1 : LibC::Char*, x2 : GpsDataT*) : LibC::Int
  fun gps_read(x0 : GpsDataT*, message : LibC::Char*, message_len : LibC::Int) : LibC::Int
  fun gps_send(x0 : GpsDataT*, x1 : LibC::Char*, ...) : LibC::Int
  fun gps_stream(x0 : GpsDataT*, x1 : LibC::UInt, x2 : Void*) : LibC::Int
  fun gps_unpack(x0 : LibC::Char*, x1 : GpsDataT*) : LibC::Int
  fun gps_waiting(x0 : GpsDataT*, x1 : LibC::Int) : Bool

  struct AisT
    type : LibC::UInt
    repeat : LibC::UInt
    mmsi : LibC::UInt
  end

  struct AttitudeT
    mtime : Timespec
    acc_len : LibC::Double
    acc_x : LibC::Double
    acc_y : LibC::Double
    acc_z : LibC::Double
    depth : LibC::Double
    dip : LibC::Double
    gyro_x : LibC::Double
    gyro_y : LibC::Double
    heading : LibC::Double
    mag_len : LibC::Double
    mag_x : LibC::Double
    mag_y : LibC::Double
    mag_z : LibC::Double
    pitch : LibC::Double
    roll : LibC::Double
    temp : LibC::Double
    yaw : LibC::Double
    mag_st : LibC::Char
    pitch_st : LibC::Char
    roll_st : LibC::Char
    yaw_st : LibC::Char
  end

  struct DevconfigT
    path : LibC::Char[128]
    flags : LibC::Int
    driver : LibC::Char[64]
    subtype : LibC::Char[128]
    hexdata : LibC::Char[512]
    activated : LibC::Double
    baudrate : LibC::UInt
    stopbits : LibC::UInt
    parity : LibC::Char
    cycle : LibC::Double
    mincycle : LibC::Double
    driver_mode : LibC::Int
  end

  struct DopT
    xdop : LibC::Double
    ydop : LibC::Double
    pdop : LibC::Double
    hdop : LibC::Double
    vdop : LibC::Double
    tdop : LibC::Double
    gdop : LibC::Double
  end

  struct GpsDataT
    set : GpsMaskT
    online : TimestampT
    gps_fd : SocketT
    fix : GpsFixT
    separation : LibC::Double
    status : LibC::Int
    satellites_used : LibC::Int
    dop : DopT
    skyview_time : TimestampT
    satellites_visible : LibC::Int
    skyview : SatelliteT[140]
    dev : DevconfigT
    policy : GpsPolicyT
    devices : GpsDataTDevices
    data_union : GpsDataTUnion
    privdata : Void*
  end

  struct GpsDataTDevices
    time : TimestampT
    ndevices : LibC::Int
    list : DevconfigT[4]
  end

  union GpsDataTUnion
    rtcm2 : Rtcm2T
    rtcm3 : Rtcm3T
    subframe : SubframeT
    ais : AisT
    attitude : AttitudeT
    navdata : NavdataT
    raw : RawdataT
    gst : GstT
    osc : OscillatorT
    version : VersionT
    error : LibC::Char[256]
    toff : TimedeltaT
    pps : TimedeltaT
  end

  struct GpsFixT
    time : TimestampT
    mode : LibC::Int
    ept : LibC::Double
    latitude : LibC::Double
    epy : LibC::Double
    longitude : LibC::Double
    epx : LibC::Double
    altitude : LibC::Double
    epv : LibC::Double
    track : LibC::Double
    epd : LibC::Double
    speed : LibC::Double
    eps : LibC::Double
    climb : LibC::Double
    epc : LibC::Double
    eph : LibC::Double
    sep : LibC::Double
    magnetic_track : LibC::Double
    ecef : GpsFixTEcef
    datum : LibC::Char[40]
    q_err : LibC::Long
  end

  struct GpsFixTEcef
    x : LibC::Double
    y : LibC::Double
    z : LibC::Double
    p_acc : LibC::Double
    vx : LibC::Double
    vy : LibC::Double
    vz : LibC::Double
    v_acc : LibC::Double
  end

  struct GpsPolicyT
    watcher : Bool
    json : Bool
    nmea : Bool
    raw : LibC::Int
    scaled : Bool
    timing : Bool
    split24 : Bool
    pps : Bool
    loglevel : LibC::Int
    devpath : LibC::Char[128]
    remote : LibC::Char[128]
  end

  struct GstT
    utctime : LibC::Double
    rms_deviation : LibC::Double
    smajor_deviation : LibC::Double
    sminor_deviation : LibC::Double
    smajor_orientation : LibC::Double
    lat_err_deviation : LibC::Double
    lon_err_deviation : LibC::Double
    alt_err_deviation : LibC::Double
  end

  struct MeasT
    gnssid : UInt8
    svid : UInt8
    sigid : UInt8
    snr : UInt8
    freqid : UInt8
    lli : UInt8
    obs_code : LibC::Char[4]
    codephase : LibC::Double
    carrierphase : LibC::Double
    pseudorange : LibC::Double
    deltarange : LibC::Double
    doppler : LibC::Double
    locktime : LibC::UInt
    l2c : LibC::Double
    c2c : LibC::Double
    satstat : LibC::UInt
  end

  struct NavdataT
    version : LibC::UInt
    compass_heading : LibC::Double
    compass_deviation : LibC::Double
    compass_variation : LibC::Double
    air_temp : LibC::Double
    air_pressure : LibC::Double
    water_temp : LibC::Double
    depth : LibC::Double
    depth_offset : LibC::Double
    wind_speed : LibC::Double
    wind_dir : LibC::Double
    crosstrack_error : LibC::Double
    compass_status : LibC::UInt
    log_cumulative : LibC::UInt
    log_trip : LibC::UInt
    crosstrack_status : LibC::UInt
  end

  struct OscillatorT
    running : Bool
    reference : Bool
    disciplined : Bool
    delta : LibC::Int
  end

  struct RawdataT
    mtime : TimespecT
    meas : MeasT[140]
  end

  struct Rtcm2T
    type : LibC::UInt
    length : LibC::UInt
    zcount : LibC::Double
    refstaid : LibC::UInt
    seqnum : LibC::UInt
    stathlth : LibC::UInt
  end

  struct Rtcm31001T
    ident : LibC::UInt
    l1 : Rtcm3BasicRtk
  end

  struct Rtcm31002T
    ident : LibC::UInt
    l1 : Rtcm3ExtendedRtk
  end

  struct Rtcm31003T
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31003TRtkData[64]
  end

  struct Rtcm31003TRtkData
    ident : LibC::UInt
    l1 : Rtcm3BasicRtk
    l2 : Rtcm3BasicRtk
  end

  struct Rtcm31004T
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31003TRtkData[64]
  end

  struct Rtcm31005T
    station_id : LibC::UInt
    system : LibC::Int
    reference_station : Bool
    single_receiver : Bool
    ecef_x : LibC::Double
    ecef_y : LibC::Double
    ecef_z : LibC::Double
  end

  struct Rtcm31006T
    station_id : LibC::UInt
    system : LibC::Int
    reference_station : Bool
    single_receiver : Bool
    ecef_x : LibC::Double
    ecef_y : LibC::Double
    ecef_z : LibC::Double
    height : LibC::Double
  end

  struct Rtcm31009T
    ident : LibC::UInt
    l1 : Rtcm3BasicRtk
  end

  struct Rtcm31010T
    ident : LibC::UInt
    l1 : Rtcm3ExtendedRtk
  end

  struct Rtcm31011T
    ident : LibC::UInt
    l1 : Rtcm3ExtendedRtk
    l2 : Rtcm3ExtendedRtk
  end

  struct Rtcm31012T
    ident : LibC::UInt
    l1 : Rtcm3ExtendedRtk
    l2 : Rtcm3ExtendedRtk
  end

  struct Rtcm31013T
    id : LibC::UShort
    sync : Bool
    interval : LibC::UShort
  end

  struct Rtcm31014T
    network_id : LibC::UInt
    subnetwork_id : LibC::UInt
    stationcount : LibC::UInt
    master_id : LibC::UInt
    aux_id : LibC::UInt
    d_lat : LibC::Double
    d_lon : LibC::Double
    d_alt : LibC::Double
  end

  struct Rtcm31015T
    header : Rtcm3NetworkRtkHeader
    corrections : Rtcm3CorrectionDiff[64]
  end

  struct Rtcm31016T
    header : Rtcm3NetworkRtkHeader
    corrections : Rtcm3CorrectionDiff[64]
  end

  struct Rtcm31017T
    header : Rtcm3NetworkRtkHeader
    corrections : Rtcm3CorrectionDiff[64]
  end

  struct Rtcm31019T
    ident : LibC::UInt
    week : LibC::UInt
    sv_accuracy : UInt8
    # code : nil
    idot : LibC::Double
    iode : UInt8
    t_sub_oc : LibC::UInt
    a_sub_f2 : LibC::Int
    a_sub_f1 : LibC::Int
    a_sub_f0 : LibC::Int
    iodc : LibC::UInt
    c_sub_rs : LibC::Int
    delta_sub_n : LibC::Int
    m_sub_0 : LibC::Int
    c_sub_uc : LibC::Int
    e : LibC::UInt
    c_sub_us : LibC::Int
    sqrt_sub_a : LibC::UInt
    t_sub_oe : LibC::UInt
    c_sub_ic : LibC::Int
    omega_sub_0 : LibC::Int
    c_sub_is : LibC::Int
    i_sub_0 : LibC::Int
    c_sub_rc : LibC::Int
    argument_of_perigee : LibC::Int
    omegadot : LibC::Int
    t_sub_gd : LibC::Int
    sv_health : UInt8
    p_data : Bool
    fit_interval : Bool
  end

  struct Rtcm31020T
    ident : LibC::UInt
    channel : LibC::UShort
    c_sub_n : Bool
    health_av_ailability_indicator : Bool
    p1 : UInt8
    t_sub_k : LibC::UShort
    msb_of_b_sub_n : Bool
    p2 : Bool
    t_sub_b : Bool
    x_sub_n_t_of_t_sub_b_prime : LibC::Int
    x_sub_n_t_of_t_sub_b : LibC::Int
    x_sub_n_t_of_t_sub_b_prime_prime : LibC::Int
    y_sub_n_t_of_t_sub_b_prime : LibC::Int
    y_sub_n_t_of_t_sub_b : LibC::Int
    y_sub_n_t_of_t_sub_b_prime_prime : LibC::Int
    z_sub_n_t_of_t_sub_b_prime : LibC::Int
    z_sub_n_t_of_t_sub_b : LibC::Int
    z_sub_n_t_of_t_sub_b_prime_prime : LibC::Int
    p3 : Bool
    gamma_sub_n_of_t_sub_b : LibC::Int
    mp : UInt8
    ml_n : Bool
    tau_n_of_t_sub_b : LibC::Int
    m_delta_tau_sub_n : LibC::Int
    e_sub_n : LibC::UInt
    mp4 : Bool
    mf_sub_t : UInt8
    mn_sub_t : UInt8
    mm : UInt8
    additioinal_data_availability : Bool
    n_sup_a : LibC::UInt
    tau_sub_c : LibC::UInt
    m_n_sub_4 : LibC::UInt
    m_tau_sub_gps : LibC::Int
    m_l_sub_n : Bool
  end

  struct Rtcm31029T
    station_id : LibC::UInt
    mjd : LibC::UShort
    sod : LibC::UInt
    len : LibC::SizeT
    unicode_units : LibC::SizeT
    text : UInt8[128]
  end

  struct Rtcm31033T
    station_id : LibC::UInt
    descriptor : LibC::Char[32]
    setup_id : LibC::UInt
    serial : LibC::Char[32]
    receiver : LibC::Char[32]
    firmware : LibC::Char[32]
  end

  struct Rtcm3BasicRtk
    indicator : UInt8
    channel : LibC::UInt
    pseudorange : LibC::Double
    rangediff : LibC::Double
    locktime : UInt8
  end

  struct Rtcm3CorrectionDiff
    ident : UInt8
    # ambiguity : Void
    nonsync : UInt8
    geometric_diff : LibC::Double
    iode : UInt8
    ionospheric_diff : LibC::Double
  end

  struct Rtcm3ExtendedRtk
    indicator : UInt8
    channel : LibC::UInt
    pseudorange : LibC::Double
    rangediff : LibC::Double
    locktime : UInt8
    ambiguity : UInt8
    cnr : LibC::Double
  end

  struct Rtcm3NetworkRtkHeader
    network_id : LibC::UInt
    subnetwork_id : LibC::UInt
    time : TimeT
    multimesg : Bool
    master_id : LibC::UInt
    aux_id : LibC::UInt
    satcount : UInt8
  end

  struct Rtcm3RtkHdr
    station_id : LibC::UInt
    tow : TimeT
    sync : Bool
    satcount : LibC::UShort
    smoothing : Bool
    interval : LibC::UInt
  end

  struct Rtcm3T
    type : LibC::UInt
    length : LibC::UInt
    rtcmtypes : Rtcm3TRtcmtypes
  end

  struct Rtcm3TRtcmtypesRtcm31001
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31001T[64]
  end

  struct Rtcm3TRtcmtypesRtcm31002
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31002T[64]
  end

  struct Rtcm3TRtcmtypesRtcm31007
    station_id : LibC::UInt
    descriptor : LibC::Char[32]
    setup_id : LibC::UInt
  end

  struct Rtcm3TRtcmtypesRtcm31008
    station_id : LibC::UInt
    descriptor : LibC::Char[32]
    setup_id : LibC::UInt
    serial : LibC::Char[32]
  end

  struct Rtcm3TRtcmtypesRtcm31009
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31009T[64]
  end

  struct Rtcm3TRtcmtypesRtcm31010
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31010T[64]
  end

  struct Rtcm3TRtcmtypesRtcm31011
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31011T[64]
  end

  struct Rtcm3TRtcmtypesRtcm31012
    header : Rtcm3RtkHdr
    rtk_data : Rtcm31012T[64]
  end

  struct Rtcm3TRtcmtypesRtcm31013
    station_id : LibC::UInt
    mjd : LibC::UShort
    sod : LibC::UInt
    leapsecs : UInt8
    ncount : UInt8
    announcements : Rtcm31013T[32]
  end

  struct SatelliteT
    ss : LibC::Double
    used : Bool
    prn : LibC::Short
    elevation : LibC::Short
    azimuth : LibC::Short
    gnssid : UInt8
    svid : UInt8
    sigid : UInt8
    freqid : UInt8
  end

  struct SubframeT
    subframe_num : Uint8T
    data_id : Uint8T
    pageid : Uint8T
    t_svid : Uint8T
    tow17 : Uint32T
    l_tow17 : LibC::Long
    integrity : Bool
    alert : Bool
    antispoof : Bool
    is_almanac : LibC::Int
  end

  struct TimedeltaT
    real : Timespec
    clock : Timespec
  end

  struct Timespec
    tv_sec : X__TimeT
    tv_nsec : X__SyscallSlongT
  end

  struct VersionT
    release : LibC::Char[64]
    rev : LibC::Char[64]
    proto_major : LibC::Int
    proto_minor : LibC::Int
    remote : LibC::Char[128]
  end

  struct X_IoFile
    _flags : LibC::Int
    _io_read_ptr : LibC::Char*
      _io_read_end : LibC::Char*
      _io_read_base : LibC::Char*
      _io_write_base : LibC::Char*
      _io_write_ptr : LibC::Char*
      _io_write_end : LibC::Char*
      _io_buf_base : LibC::Char*
      _io_buf_end : LibC::Char*
      _io_save_base : LibC::Char*
      _io_backup_base : LibC::Char*
      _io_save_end : LibC::Char*
      _markers : X_IoMarker*
      _chain : X_IoFile*
      _fileno : LibC::Int
    _flags2 : LibC::Int
    _old_offset : X__OffT
    _cur_column : LibC::UShort
    _vtable_offset : LibC::Char
    _shortbuf : LibC::Char[1]
    _lock : X_IoLockT*
      _offset : X__Off64T
    _codecvt : X_IoCodecvt*
      _wide_data : X_IoWideData*
      _freeres_list : X_IoFile*
      _freeres_buf : Void*
      __pad5 : LibC::SizeT
    _mode : LibC::Int
    _unused2 : LibC::Char[20]
  end

  type File = X_IoFile
  type TimespecT = Timespec

  union Rtcm3TRtcmtypes
  rtcm3_1001 : Rtcm3TRtcmtypesRtcm31001
  rtcm3_1002 : Rtcm3TRtcmtypesRtcm31002
  rtcm3_1003 : Rtcm31003T
  rtcm3_1004 : Rtcm31004T
  rtcm3_1005 : Rtcm31005T
  rtcm3_1006 : Rtcm31006T
  rtcm3_1007 : Rtcm3TRtcmtypesRtcm31007
  rtcm3_1008 : Rtcm3TRtcmtypesRtcm31008
  rtcm3_1009 : Rtcm3TRtcmtypesRtcm31009
  rtcm3_1010 : Rtcm3TRtcmtypesRtcm31010
  rtcm3_1011 : Rtcm3TRtcmtypesRtcm31011
  rtcm3_1012 : Rtcm3TRtcmtypesRtcm31012
  rtcm3_1013 : Rtcm3TRtcmtypesRtcm31013
  rtcm3_1014 : Rtcm31014T
  rtcm3_1015 : Rtcm31015T
  rtcm3_1016 : Rtcm31016T
  rtcm3_1017 : Rtcm31017T
  rtcm3_1019 : Rtcm31019T
  rtcm3_1020 : Rtcm31020T
  rtcm3_1029 : Rtcm31029T
  rtcm3_1033 : Rtcm31033T
  data : UInt8[1024]
end
end

