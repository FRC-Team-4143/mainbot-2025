package frc.mw_lib.proxy_server;

public interface Packet {
  // Timestamp class to store packet timestamp and preform operations

  // Byte index for packet header data
  public static final int ID_IDX = 0;
  public static final int TIME_SEC_IDX = 1;
  public static final int TIME_NSEC_IDX = 5;

  public static class Timestamp {
    public int seconds;
    public int nanoseconds;

    public Timestamp(int seconds, int nanoseconds) {
      this.seconds = seconds;
      this.nanoseconds = nanoseconds;
    }

    /**
     * Determines is current timestamp is more recent than supplied timestamp
     *
     * @param recorded the store {@link #Timestamp} to compare current timestamp against
     * @return true: current timestamp is the most recent | false: recorded timestamp is the most
     *     recent
     */
    public boolean isLatest(Timestamp recorded) {
      if (seconds > recorded.seconds) {
        return true;
      } else if (nanoseconds > recorded.nanoseconds) {
        return true;
      } else {
        return false;
      }
    }
  }
}
