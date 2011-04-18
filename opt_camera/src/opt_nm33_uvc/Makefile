TARGET=init_xu_register libopt_nm33_camera.so opt_nm33_viewer
all: $(TARGET)

.cpp.o:
	gcc -o $@ -c $< -fPIC `pkg-config opencv --cflags`

init_xu_register: init_xu_register.o opt_nm33_uvc.o
	g++ -o $@ $^

opt_nm33_viewer: opt_nm33_viewer.o libopt_nm33_camera.so
	g++ -o $@ $^ -L./ -lopt_nm33_camera

libopt_nm33_camera.so: opt_nm33_uvc.o opt_nm33_camera.o
	g++ --shared -o $@ $^ `pkg-config opencv --libs`

clean:
	rm -f *.o *.so *~ $(TARGET)

FILENAME=opt_nm33_uvc-0.1.tgz
release:
	tar -C ../ -cvzf /tmp/$(FILENAME) ./opt_nm33_uvc --exclude ".svn" --exclude ".~"
upload:
	scp /tmp/$(FILENAME) k-okada@www.jsk.t.u-tokyo.ac.jp:public_html/
