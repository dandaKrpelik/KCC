import numpy as np
import matplotlib.pyplot as plt


def chooseK(n,k, exept = None):
	x = np.random.permutation(n)
	
	if not exept is None:
		for i in range(k):
			if x[i] == exept:
				x[i] = x[k]
				break
	return x[:k]

class DE:
	def __init__(self, fnc, npop, dim, bounds):
		self.pop = np.zeros((npop, dim))
		self.fnc = fnc
		self.fit = np.zeros(npop)
		self.bounds = bounds
	
		self.npop = npop
		self.dim = dim
	
		for i in range(npop):
			x = self.getIndividual()
			self.pop[i,:] = x
			self.fit[i] = fnc(x)
			
		self.best = self.getBest()
		self.allbest = self.best
			
		self.log = []
		self.logMe()
	
	def getIndividual(self):
		xout = np.random.randn(self.dim)
		xout = self.fixbounds(xout)			
		return xout
	
	def fixbounds(self,x):
		bounds = self.bounds
		for i in range(self.dim):
			if x[i] < bounds[i][0]:
				x[i] = bounds[i][0]+1e-5
			if x[i] > bounds[i][1]:
				x[i] = bounds[i][1]	-1e-5
				
		return x	
	
	def getBest(self):
		order = [[self.fit[i], self.pop[i]] for i in range(self.npop)]
		order.sort(key = lambda x:x[0])
		return order[0]		
	
	def logMe(self):
		self.best = self.getBest()
		if self.best[0] < self.allbest[0]:
			self.allbest = self.best
		l = [min(self.fit), np.mean(self.fit), self.best[0]]
		self.log.append(l)
	
	def show(self):
		plt.figure()
		plt.plot([x[1] for x in self.log], label = 'mean')
		plt.plot([x[0] for x in self.log], label = 'min')
		plt.plot([x[2] for x in self.log], label = 'best')
		
		plt.axhline([self.allbest[0]])
		plt.legend()
		plt.show(0)
	
	def iterate(self, maxit = 1):
		pop = self.pop
		fit = self.fit
		npop = self.npop
		dim = self.dim
		
		F = 2.319
		Cr = 0.6
		mut = 0.1
		
		for it in range(maxit):
#			print('\rOptimizing, iterace : \t %4d \t of %4d' % (it, maxit), end='\r')
			newpop = np.zeros((2*npop,dim))
			newpop[:npop,:] = pop
			newfit = np.zeros(2*npop)
			newfit[:npop] = fit
			
			for i in range(npop):
				chosen = chooseK(npop, 3, i)
				a = chosen[0]; b = chosen[1]; c=chosen[2]
				diff = pop[a,:] + F*(pop[b,:] - pop[c,:])
				r = np.random.rand(dim)
				for j in range(dim):
					if r[j] < Cr:
						diff[j] = pop[i,j]
				
				
				#diff = np.abs(diff)
				diff = self.fixbounds(diff)
				newpop[i+npop,:] = diff
				newfit[i+npop] = self.fnc(diff)
			
			b = [[x,i] for i,x in enumerate(newfit)]
			b.sort()
			
			for i in range(npop):
				index = b[i][1]
				pop[i,:] = newpop[index,:]
				fit[i] = newfit[index]
				if np.random.rand() < mut and i > 0:
					pop[i,:] = self.getIndividual()
					fit[i] = self.fnc(pop[i,:])
			
			
			self.logMe()
			
			
#		print('\n')
		
