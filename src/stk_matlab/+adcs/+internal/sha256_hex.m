function hex = sha256_hex(bytes)
%SHA256_HEX Lower-case hex sha256 of a uint8 vector.
    digest = java.security.MessageDigest.getInstance('SHA-256');
    if ~isempty(bytes)
        digest.update(typecast(uint8(bytes(:)), 'int8'));
    end
    raw = typecast(digest.digest(), 'uint8');
    hex = lower(reshape(dec2hex(raw, 2).', 1, []));
end
