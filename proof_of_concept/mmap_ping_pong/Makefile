all: mping

mping: mping.cpp
	g++ $< -o $@

test: mping
	./mping 1 100001 &
	time ./mping 0 100000

clean:
	-rm mping
