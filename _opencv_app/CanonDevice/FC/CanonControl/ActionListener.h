#pragma once

class ActionEvent;
class ActionListener{
	public:
	virtual void actionPerformed(const ActionEvent& event) =0;
};