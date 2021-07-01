# tf_relay

This node broadcasts transforms from a given fixed frame to a given output frame which is the same as reference frame in specified duraiton even when reference frame is not broadcasted.


## Parameters

- '~max_duration' ( double, default: 5.0 )

cache duration for tf2 buffer.

- '~timeout_duration' ( double, default: 0.05 )

timeout duration for lookuptransform of tf.

- '~timer_duration' ( double, default: 0.1 )

spin duration for main process

- '~reference_frame_id" ( string, default: 'reference_frame' )

tf frame_id of reference frame.

- '~output_frame_id' ( string, default: 'output_frame' )

tf frame_id of output frame.

- '~fixed_frame_id' ( string, default: 'fixed_frame' )

tf frame_id of fixed frame.
