# BOLT: A Stateful Processor Interconnect

<http://bolt.ethz.ch>

In order to address the ever increasing demands of IoT, we advocate a new architectural blueprint for the design of composable and predictable multi-processor wireless sensing platforms.

BOLT is an ultra-low power processor interconnect that decouples arbitrary application and communication processors with respect to time, power and clock domains. BOLT supports asynchronous message passing with predictable timing characteristics, and therefore making it possible for the system designer to construct highly-customized platforms that are easier to design, implement, debug, and maintain.

In this repository, we provide an implementation of the BOLT processor interconnect for the 16-bit TI MSP430FR5969 microcontroller.


## Further Reading

If you want to learn more about BOLT, please have a look at our [SenSys'15 paper](http://www.tik.ee.ethz.ch/file/5acae22d79f04e0e9eb1022d089029d0/SZDLGGFBT2015a.pdf).


## Code

The code was developed for the MSP430FR5969 microcontroller and can be compiled with the [TI Code Composer Studio](http://www.ti.com/tool/CCSTUDIO).
To customize BOLT, edit the parameters in the config.h file. Parameters that can be changed include the maximum message size, the number of elements in each of the two FIFO queues and whether or not the queue should be cleared after a reset. If you don't need to tune specific parameters, there are also a few precompiled BOLT binaries in the designated folder.


## License

The BOLT code is released under [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause). For more details please refer the the [LICENSE](LICENSE) file.


## Contributors

BOLT was developed at the [Computer Engineering Group](http://www.tec.ethz.ch/) at [ETH Zurich](https://www.ethz.ch/en.html). The following people contributed to the design and implementation: [Felix Sutton](http://www.tik.ee.ethz.ch/~fsutton), [Marco Zimmerling](http://www.tik.ee.ethz.ch/~marcoz/), [Reto Da Forno](http://ch.linkedin.com/in/rdaforno), [Roman Lim](https://www.linkedin.com/in/roman-lim-42b0a312b/), [Tonio Gsell](https://github.com/tgsell), [Georgia Giannopoulou](https://www.linkedin.com/in/georgiagiannopoulou), [Jan Beutel](http://www.tik.ee.ethz.ch/~beutel), [Federico Ferrari](https://ch.linkedin.com/in/fferrari) and [Lothar Thiele](http://www.tik.ee.ethz.ch/~thiele).
