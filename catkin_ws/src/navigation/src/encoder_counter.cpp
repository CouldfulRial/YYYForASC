// Includes required for threading
#include <mutex>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <gpiod.h>

// Declare the member variables
// > For sharing the encoder counts between threads
int m_counts_for_m1a = 0;
int m_counts_for_m1b = 0;
int m_counts_for_m2a = 0;
int m_counts_for_m2b = 0;
// > Mutex for preventing multiple-access of shared variables
std::mutex m_counting_mutex;
// > Boolean flag for when to stop counting
bool m_threads_should_count = true;

// Declare the function prototypes
// > For the thread that reads (and displays) the current counts
void threadForReading();
// > For the thread that counts encoder events
void threadForCounting();



// Function implementation
void threadForCounting()
{
  // Specify the gpio chip name of the GPIO interface
  const char * gpio_chip_name = "/dev/gpiochip1";

  // Specify the line numbers where the encoder channels are connected
  int line_number_for_m1a = 105;
  int line_number_for_m1b = 106;
  int line_number_for_m2a =  84;
  int line_number_for_m2b = 130;

  // Initialise a GPIO chip, line, and event objects
  struct gpiod_chip *chip;
  struct gpiod_line *line_m1a;
  struct gpiod_line *line_m1b;
  struct gpiod_line *line_m2a;
  struct gpiod_line *line_m2b;
  struct gpiod_line_bulk line_bulk;
  struct gpiod_line_event event;
  struct gpiod_line_bulk event_bulk;

  // Specify the timeout specifications
  // > The first entry is seconds
  // > The second entry is nano-seconds
  struct timespec timeout_spec = { 0, 10000000 };

  // Intialise a variable for the flags returned by GPIO calls
  int returned_wait_flag;
  int returned_read_flag;

  // Open the GPIO chip
  chip = gpiod_chip_open(gpio_chip_name);
  // Retrieve the GPIO lines
  line_m1a  = gpiod_chip_get_line(chip,line_number_for_m1a);
  line_m1b  = gpiod_chip_get_line(chip,line_number_for_m1b);
  line_m2a = gpiod_chip_get_line(chip,line_number_for_m2a);
  line_m2b = gpiod_chip_get_line(chip,line_number_for_m2b);
  // Initialise the line bulk
  gpiod_line_bulk_init(&line_bulk);
  // Add the lines to the line bulk
  gpiod_line_bulk_add(&line_bulk, line_m1a);
  gpiod_line_bulk_add(&line_bulk, line_m1b);
  gpiod_line_bulk_add(&line_bulk, line_m2a);
  gpiod_line_bulk_add(&line_bulk, line_m2b);

  // Display the status
  std::cout << "[ENCODER COUNTER] Chip " << gpio_chip_name << " opened and lines " << line_number_for_m1a << ", " << line_number_for_m1b << ", " << line_number_for_m2a << " and " << line_number_for_m2a << " retrieved";

  // Request the line events to be monitored
  // > Note: only one of these should be uncommented
  gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
  //gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
  //gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");


  // Enter a loop that monitors the encoders
  while (m_threads_should_count)
  {
    // Monitor for the requested events on the GPIO line bulk
    // > Note: the function "gpiod_line_event_wait" returns:
    //    0  if wait timed out
    //   -1  if an error occurred
    //    1  if an event occurred.
    returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);

    // Lock the mutex before processing the events
    m_counting_mutex.lock();

    // Respond based on the the return flag
    if (returned_wait_flag == 1)
    {
      // Get the number of events that occurred
      int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

      // Iterate over the event
      for (int i_event = 0; i_event < num_events_during_wait; i_event++)
      {
        // Get the line handle for this event
        struct gpiod_line *line_handle = gpiod_line_bulk_get_line(&event_bulk, i_event);

        // Get the number of this line
        unsigned int this_line_number = gpiod_line_offset(line_handle);

        // Read the event on the GPIO line
        // > Note: the function "gpiod_line_event_read" returns:
        //    0  if the event was read correctly
        //   -1  if an error occurred
        returned_read_flag = gpiod_line_event_read(line_handle,&event);

        // Respond based on the the return flag
        if (returned_read_flag == 0)
        {
          // Increment the respective count
          if (this_line_number == line_number_for_m1a)
            m_counts_for_m1a++;
          else if (this_line_number == line_number_for_m1b)
            m_counts_for_m1b++;
          else if (this_line_number == line_number_for_m2a)
            m_counts_for_m2a++;
          else if (this_line_number == line_number_for_m2b)
            m_counts_for_m2b++;

        } // END OF: "if (returned_read_flag == 0)"

      } // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

    } // END OF: "if (returned_wait_flag == 1)"

    // Get a local copy of the flag
    bool should_count_local_copy = m_threads_should_count;

    // Unlock the mutex
    m_counting_mutex.unlock();

    // Break if necessary
    if (!should_count_local_copy)
      break;

  } // END OF: "while (true)"

  // Release the lines
  gpiod_line_release_bulk(&line_bulk);
  // Close the GPIO chip
  gpiod_chip_close(chip);
  // Inform the user
  std::cout << "[ENCODER COUNTER] Lines released and GPIO chip closed";

} // END OF: "void threadForCounting()"



// Function implementation
void threadForReading()
{
  static int cumulative_counts_for_m1 = 0;
  static int cumulative_counts_for_m2 = 0;

  while(m_threads_should_count)
  {
    // Get a local copy of the variables
    m_counting_mutex.lock();
    int counts_for_m1a_local_copy = m_counts_for_m1a;
    int counts_for_m1b_local_copy = m_counts_for_m1b;
    int counts_for_m2a_local_copy = m_counts_for_m2a;
    int counts_for_m2b_local_copy = m_counts_for_m2b;
    bool should_count_local_copy  = m_threads_should_count;
    // Reset the shared counting variables to zero
    m_counts_for_m1a = 0;
    m_counts_for_m1b = 0;
    m_counts_for_m2a = 0;
    m_counts_for_m2b = 0;
    m_counting_mutex.unlock();

    // Add the counts to the cumulative total
    cumulative_counts_for_m1 += (counts_for_m1a_local_copy + counts_for_m1b_local_copy);
    cumulative_counts_for_m2 += (counts_for_m2a_local_copy + counts_for_m2b_local_copy);

    // Display the cumulative counts
    printf("cumulative counts = [%d, %d]\n", cumulative_counts_for_m1, cumulative_counts_for_m2);

    // Break if necessary
    if (!should_count_local_copy)
      break;

    // Sleep for a bit
    usleep(100000);
  }
}



// Main function
int main(int argc, char* argv[])
{
  // Set the flag that threads should count
  m_threads_should_count = true;

  // Create the threads
  std::thread counting_thread (threadForCounting);
  std::thread reading_thread (threadForReading);

  // Spin the main function for a bit
  sleep(3);

  // Set the flag to finish counting
  m_counting_mutex.lock();
  m_threads_should_count = false;
  m_counting_mutex.unlock();

  // Join back the threads
  counting_thread.join();
  reading_thread.join();
}