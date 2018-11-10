from threading import Timer

class MyTimer:
	
	def __init__(self):
		self._timer= None
		self._tm = None
		self._fn = None
	
	def _do_func(self):
		if self._fn:
			self._fn()
			self._do_start()

	def _do_start(self):
		self._timer = Timer(self._tm, self._do_func)
		self._timer.start()

	def start(self, tm, fn):
		self._fn = fn
		self._tm = tm
		self._do_start()

	def stop(self):
		try:
			self._timer.cancel()
		except:
			pass
			
def hello():
	from datetime import datetime
	print("hello world!", datetime.now())


if __name__ == '__main__':
       
        mt = MyTimer()
        mt.start(2, hello)
        for i in range(10):
            import time
            print("stop")
            time.sleep(2)
            
        mt.stop()
