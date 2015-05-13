-- silverhammer protocol plugin for wireshark
-- protocol definition
silverhammer_proto = Proto("silverhammer", "SILVERHAMMER", "Silverhammer Protocol")
-- parse function definition
function silverhammer_proto.dissector(buffer,pinfo,tree)
    pinfo.cols.protocol = "SILVERHAMMER"
    pinfo.cols.info = "seq: " .. buffer(0,4):uint() .. ", idx: " .. buffer(4,4):uint() .. ", num: " .. buffer(8,4):uint()
    local subtree = tree:add(silverhammer_proto,buffer(),"Silverhammer Protocol Data")
    subtree:add(buffer(0,4),"Sequence ID: " .. buffer(0,4):uint())
    subtree:add(buffer(4,4),"Packet ID: " .. buffer(4,4):uint())
    subtree:add(buffer(8,4),"Packet Num: " .. buffer(8,4):uint())
    subtree:add(buffer(12),"Data: " .. buffer(12))
end
-- protocol bind registration
udp_table = DissectorTable.get("udp.port")
udp_table:add(16485,silverhammer_proto)
udp_table:add(703,silverhammer_proto)
udp_table:add(1024,silverhammer_proto)
