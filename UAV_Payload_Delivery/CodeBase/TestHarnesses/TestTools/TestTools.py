import math
import argparse
import pickle

import re

"""
Modes are:
	
RUN:  run procedure for grading or testing purposes
GENERATE: run procedure on working code to generate (presumably) ideal outputs
"""

"""verbosity levels are"""
SUMMARY = 2
DETAIL = 5
DEBUG = 8

verb_dict = {"SUMMARY":SUMMARY, 
			 "DETAIL":DETAIL, 
			 "DEBUG":DEBUG}

default_verbosity_level = "SUMMARY"
global_verbosity_level = verb_dict[default_verbosity_level]

black='\033[30m'
red='\033[31m'
green='\033[32m'
orange='\033[33m'
blue='\033[34m'
purple='\033[35m'
cyan='\033[36m'
lightgrey='\033[37m'
darkgrey='\033[90m'
lightred='\033[91m'
lightgreen='\033[92m'
yellow='\033[93m'
lightblue='\033[94m'
pink='\033[95m'
lightcyan='\033[96m'
white='\033[37m'


#%% Argparse:

def parse_args_for_lab(lab_name):
	parser = argparse.ArgumentParser(description =f""" A grading harness for {lab_name}.  We're trying out a new system for test harnesses.  If you have issues or suggestions on how it could be improved, please let MaxL know via Piazza.""")
	
	parser.add_argument("-g", "--generate", action = "store_true",
						help=f"Use current code to generate expected values, overwritng the pickle files.  {red}STUDENTS DON'T EVER WANT TO USE THIS OPTION!{white}")
	
	parser.add_argument("-v", "--verbosity", default=default_verbosity_level,
						help=f"How much detail to print.  Currently options are {[k for k in verb_dict.keys()]}.  Default is {default_verbosity_level}")
	
	parser.add_argument("-e", "--error_skipping", action="store_true",
						help="By default, errors generated during testing are raised to the top level, halting the test harness's execution.  This option specifies that the test harness should continue running, instead.")
	
	parser.add_argument("-t", "--tests_to_run",default = None,
						help="""A regex to select specific tests.  If used, only tests whose name matches this regex string will run.""")
	
	parser.add_argument("--no_color", action = "store_true",
						help="""Print with no color flags""")
	
	return parser
	
#%% global actions for args:
	
def apply_args(args):
	if args.no_color:
		global black, red, green, orange, blue, purple, cyan, lightgrey, darkgrey, lightred, lightgreen, yellow, lightblue, pink, lightcyan, white 
		black, red, green, orange, blue, purple, cyan, lightgrey, darkgrey, lightred, lightgreen, yellow, lightblue, pink, lightcyan, white =  "", "","", "", "", "", "", "", "", "", "", "", "", "", "", ""
#%% logging controls:
def set_global_verbosity_level(level_name):
	global global_verbosity_level
	print(f"{lightblue}verbosity set to {level_name}, see --help for more info{white}")
	level = verb_dict[level_name]
	global_verbosity_level = level
# 	print(f"{lightblue}level = {global_verbosity_level}{white}")
	
def ttprint(level, to_print):
# 	print(f"level={level}, global_verbosity_level={global_verbosity_level}")
	if level <= global_verbosity_level:
			print(to_print)

#%% various helpers:
	
def rel_err(obs, exp):
	if obs == exp:
		return 0
	elif abs(exp) < 1e-15:  
# 		return abs(obs) #Dunno what to do for divbyzero, I guess just absolute error
		return math.nan
	else:
		return ((exp-obs)/abs(exp))
	
def sigfig(x, digits=6, epsilon = 1e-9):
	if x == 0 or not math.isfinite(x):
		return x
    # digits -= math.ceil(math.log10(abs(x)))
	# return round(x, digits)
	return round(x, digits - int(math.floor(math.log10(max(abs(x), abs(epsilon))))) - 1)

def deep_compare(name, obs, exp, rel_tol = 0, abs_tol = 0):
	#first, try recursing:
	try:
		if len(obs) != len(exp):
			raise Exception(f"tried to compare objects with different len()s:  obs={obs}, exp={exp}")
		recursed = [deep_compare(f"{name}_{i}", ai, bi, rel_tol=rel_tol,abs_tol=abs_tol)
				  for i, (ai, bi) in enumerate(zip(obs,exp))]
		return all(recursed)
	except TypeError: #this happens if there's no length, it's our recursive base case
		try:
			passed= math.isclose(obs,exp, abs_tol = abs_tol, rel_tol = rel_tol)
		except Exception as e:
			e_str = f"error when comparing results for output parameter {name}:  obs={obs}, exp={exp}.   The exception was: {e}"
			raise Exception(e_str)
		if passed:
			ttprint(DEBUG, f"      {name:>10}: exp={exp:>13e} {green}=={white} obs={obs:>13e} ({rel_err(obs,exp):>13e})")
		else:
			ttprint(DEBUG, f"      {name:>10}: exp={exp:>13e} {red}!={white} obs={obs:>13e} ({rel_err(obs,exp):>13e})")
		return passed
		
def deep_rel_err(obs,exp):
	#first, try recursing:
	try:
		if len(obs) != len(exp):
			raise Exception(f"tried to compare object with len {len(obs)} to object with len {len(exp)}")
		recursed = [deep_rel_err(ai, bi)
				  for ai, bi in zip(obs,exp)]
# 		print(recursed)
		return (sum(recursed))
	except TypeError: #this happens if there's no length, it's our base case
		return abs(rel_err(obs,exp))
	except ValueError: #this happens if there's a nan.  There is probably a better way to handle these but who knows?
		return math.nan
		

#testing deep_compare
if __name__=="__main__":
	assert( deep_compare("test", 4, 4) )
	assert( deep_compare("test",[3,3,1], [3,3,1]) )
	assert( deep_compare("test",[[3, 4],3,1], [[3,4] ,3,1]) )
	assert( not deep_compare("test", 4, 5) )
	assert( not deep_compare("test",[3,3,1], [3,5,1]) )
	assert( not deep_compare("test",[[3, 4],3,1], [[3,5] ,3,1]) )


def d2r(deg):
	"""degrees to radians.  I guess it's the same as math.radians, which I didn't know about when I wrote this function...."""
	return deg * math.pi / 180

def round_dict(dic, digits=2):
	"""we often wish to have dicts that are a bit more visually parseable, so this just..rounds them"""
	for k,v in dic.items():
		dic[k] = sigfig(v, digits)
	return dic


#%%		
class TestManager:
	def __init__(self, lab_name, args):
		self.mode = "GENERATE" if args.generate else "RUN"
		
		self.error_skipping = args.error_skipping
		
		set_global_verbosity_level(args.verbosity)
		
		self.outcomes = {} #store outcomes of all tests
		self.rel_errs = {} #store relative errors of failed testsOB
		self.all_blocks = {} #also store outcomes by block

		self.picklePath = f'{lab_name}_data.pickle'
		self.tests_to_run = args.tests_to_run

		if self.mode == "GENERATE":
			self.results = {}
		else:
			ttprint(DEBUG, f"Loading file '{self.picklePath}'...")
			with open(self.picklePath, 'rb') as f:
				self.results = pickle.load(f)
			first_tests = [k for k in self.results.keys()][0:3]
			ttprint(DEBUG, f"loaded {len(self.results)} tests starting with {first_tests}")
# 		tt.ttprint(tt.DEBUG, f"Results dict is {expected_results_dict}")
	
		
	def test(self, name, procedure, inputs,
			  rel_tol = 1e-12, abs_tol= 1e-12):
		"""
		This function will do one of two things, depending on which mode it runs in
		
		name:  String.  A key-friendly name for the test. Tests should have unique names, as they serve as keys in the results dict
		procedure: function (ie a 'function pointer'), which must take in a single argument, "inputs", and must output a dict.  Input can in theory be whatever, as long as it has a __str__ method, but I'd recommend dicts, since they force a nice specificity on you, the test designer.
		
		"""
		if self.mode == "RUN":
			if self.tests_to_run:
				if not re.search(self.tests_to_run, name):
					return
			
			ttprint(DEBUG, f"   Running test {name} with inputs {inputs}...")
# 			for k, v in inputs.items():
# 				ttprint(DEBUG, f"\n    {k:10}:{v}")
			expected_outputs = self.results[name]
			
			#run student code:
			try:
				observed_outputs = procedure(inputs)
				outcome, err = self.compare_outputs(
					observed_outputs, expected_outputs, 
					rel_tol = rel_tol, abs_tol=abs_tol)
			except Exception as e:
				if self.error_skipping:
					ttprint(DETAIL, str(e))
					outcome = False
					err = 999999
				else:
					raise e
			
			self.outcomes[name] = outcome
			self.rel_errs[name] = err
			if not self.tests_to_run:
				self.cur_block_results[name] = outcome
			if outcome:
				ttprint(DEBUG, f"   {green}test {name} passed{white}")
			else:
				ttprint(DETAIL, f"   {red}test {name} failed{white}")
			ttprint(DEBUG,"")#just here for the  newline 
		elif self.mode == "GENERATE":
			
			if name in self.results.keys():
				raise Exception(f"Key '{name}' is already in results dict!  Instances of Tests or Test children need unique names.")
			ttprint(DEBUG, f"   Running test {name} with inputs:")
			for k, v in inputs.items():
				ttprint(DEBUG, f"    {k:10}:{v}")
			outputs = procedure(inputs)
			ttprint(DEBUG, f"      Got outputs {outputs}")
			self.results[name] = outputs
			
	def start_block(self, block_name):
		"""
		Used to break test results into more readable chunks.  This function starts a new block.
		"""
		if self.tests_to_run:  #just don't use blocks when in this mode
			return
		self.cur_block_name = block_name
		ttprint(SUMMARY, f"{cyan}Beginning test section {self.cur_block_name}{white}")
		self.cur_block_results = {}
		
	def end_block(self):
		"""
		Used to break test results into more readable chunks.  This method ends a block, causing a short summary to be printed.
		"""
		if self.tests_to_run: #just don't use blocks when in this mode
			return
		if not self.mode=="RUN":
			return
		ttprint(DETAIL, f"  results for block {self.cur_block_name}:")
# 		for name, outcome in self.cur_block_results.items():
# 			if not outcome:
# 				ttprint(DETAIL, f"  Failed test {red}{name}{white}")
		passed = sum(self.cur_block_results.values())
		total = len(self.cur_block_results)
		color = green if passed==total else (
			orange if passed >= math.ceil(0.1*total) else red)
		ttprint(SUMMARY, f"  {color} passed {passed}/{total} tests {white}")
		
		self.all_blocks[self.cur_block_name] = self.cur_block_results
		
	def compare_outputs(self, observed, expected, rel_tol = 1e-9, abs_tol=1e-9):
		#sanity check, real and expected should have same keys:
		obs_set = set(observed.keys())
		exp_set = set(expected.keys())
		if obs_set != exp_set:
			#first, ID which keys are missing:
			raise Exception(f"""observed and expected outputs have different keys! 
				   
observed={observed.keys()}

expected={expected.keys()}

obs-exp={obs_set-exp_set}

exp-obs={exp_set-obs_set}""")
		
		#and otherwise all is good
		pass_list = []
		err_list = []
		ttprint(DEBUG,"   Outputs:")
		for key, expected_val in expected.items(): 
			observed_val = observed[key]
# 			passed = math.isclose(expected_val,observed_val, 
# 						 rel_tol = rel_tol, abs_tol = abs_tol)
			passed = deep_compare(key, observed_val, expected_val,
						 rel_tol = rel_tol, abs_tol = abs_tol)
			err = deep_rel_err(observed_val, expected_val)
			pass_list.append(passed)
			err_list.append(err)
# 			if passed:
# 				ttprint(DEBUG, f"      {key}: exp={expected_val} {green}=={white} obs={observed_val}  ")
# 			else:
# 				ttprint(DEBUG, f"      {key}: exp={expected_val} {red}!={white} obs={observed_val}  ")
				
		if all(pass_list):
			return (True, sum(err_list))
		else:
			return (False, sum(err_list))
		
	def conclude(self):
		if self.mode == "RUN":
			ttprint(SUMMARY, "Test Harness complete, printing summary:")
			self.print_run_summary()
			
		if self.mode == "GENERATE":
			ttprint(SUMMARY, f"{green}Test battery complete,{white} dumping results into {self.picklePath}...")
			with open(self.picklePath, 'wb') as f:
				pickle.dump(self.results, f)
			self.print_generate_summary()
		
	def print_run_summary(self):
		passes = 0
		fails = 0

		for testname, outcome in self.outcomes.items():
			if outcome == True:
				passes += 1 
			else:
				fails += 1
		if fails == 0:
			ttprint(0, f"{green} passed all {passes} tests!{white}")
		else:
			ttprint(0, f"{orange} passed {passes}/{passes+fails} tests, failed {fails}/{passes+fails} tests{white}")
			ttprint(DETAIL, "List of failed tests:")
			for name, outcome in self.outcomes.items():
				if not outcome:
					err = self.rel_errs[name]
					ttprint(DETAIL, f"{red}{name:40}{white} ({err:e})")
					
	def print_generate_summary(self):
			ttprint(DETAIL, "All tests:")
			[ttprint(DETAIL, "   "+test) for test in self.results.keys()]
			
			
	def store_pregen(self, key, value):
		"""sometimes it's useful to store non-test things during the generation step (particularly useful for things that are generated by student code, like transfer fcns or trim states). Since we're pickling a dict anyway, this is easy."""
		self.results[key] = value
		
	def retrieve_pregen(self, key):
		return self.results[key]			
			
#%%
