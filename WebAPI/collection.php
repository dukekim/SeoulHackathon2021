<?php

	//----------------------------------------------------------------------------------
	//	Input Date
	//----------------------------------------------------------------------------------
	$id		= $_REQUEST['id'];
	$action = $_REQUEST['action'];
	$value1	= $_REQUEST['value1'];
	$value2	= $_REQUEST['value2'];
	$value3	= $_REQUEST['value3'];	
			
	//------------------------------------------------------------------------------
	//	Insert Data Base
	//------------------------------------------------------------------------------	
		
	
	$db = new PDO("sqlsrv:Server=DB Your IP;Database=Your DB  Name", "DB ID", "DB Password");
	if( !$db ) return;
	
	$data = [
		'id'		=> $id,
		'action' 	=> $action,		
		'value1' 	=> $value1,
		'value2' 	=> $value2,
		'value3' 	=> $value3

	];
	
	
	$sql = "INSERT INTO data ( data_id, data_action, data_value1, data_value2, data_value3) VALUES ( :id, :action, :value1, :value2, :value3 )";
	$stmt= $db->prepare($sql);	
	$stmt->execute($data);
	
	
?>