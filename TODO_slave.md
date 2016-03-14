# Ideas for a CanOpen slave library

* Where? Fits best in master library (rename to node e.g.), because it shares many things like dictionary and eds_reader,
	and master could inherit from the slave class (-> inlining).

* Core::received_message:
	- implement two missing cases in receive loop (simple delegation to sdo/pdo class)

* PDO class:
	- Not much to do here. Just provide callbacks for RPDOs. These will be registered by Slave class.
	- Rename process_incoming_message() to process_incoming_tpdo() and add process_incoming_rpdo()
	- Rename add_pdo_received_callback() to add_tpdo_received_callback() and add add_rpdo_received_callback()	

* SDO class:
	- Just like in PDO class, we need to implement client SDOs.
	- Slave class listens for requests and (add_request_callback()) and can react using send_response()
	- add add_client_sdo_callback(SDOReceivedCallback) (callback signature void(const SDOResponse&)) and/or
		add_request_callback(node_id, SDORequestCallback) (listening only for SDOs with slave's own node_id, callback signature void(index, subindex))
	- add send_response(node_id, index, subindex, vector<uint8_t> data) (chooses segmented/expedited transfer on it's own)
	- add abort_transfer(node_id, index, subindex, errorcode)

* Slave class:
	- has a node_id! (never used this until now...)
	- m_dictionary of type map<name, Entry>
	- m_index_to_name of type map<index,subindex,name>
	- add_entry(Entry)
	- read_dictionary_from_eds(filename)
		-> store default values
	- set_value(name,value)
		-> user should call this for all entries without default value
	- get_value(name) (internal use)
	- register sdo_request_callback
		-> get name from index, do get_value() and send result using core.sdo.send_response(node_id, index, subindex, value.get_bytes())
	- add_pdo_mapping(cob_id, name)
		-> core.pdo.add_rpdo_received_callback() with this cob_id -> set_value()...  

* Master class:
	- inherit from slave
	- add at least mandatory entries using add_entry() in constructor

* EDSReader class:
	- handle default values but only if used by Slave class, otherwise dictionary could contain outdated values.