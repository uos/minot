// Implementation of the protocol over TCP.
// First value is u64 representing the length of the message.
// Second value is flag what type of message it is.
// Types (always sending): 0 = Vector<f32>, 1 = Vector<f64>, 2 = Matrix<f32> (column major), 3 = Matrix<f64> (column major)
